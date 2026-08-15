/**
  ******************************************************************************
  * @file    DRV8251A.c
  * @brief   Source file for the DRV8251A H-Bridge Motor Driver Library.
  *          Implements speed, direction, braking, fault checking, and current sensing.
  * @author  Antigravity
  * @date    August 2026
  ******************************************************************************
  */
#include "main.h"
#include "DRV8251A.h"

/* The IPROPI current mirror scaling factor is typically 1575 uA/A = 0.001575 A/A */
#define A_IPROPI_FACTOR 0.001575f

/**
  * @brief  Initializes the DRV8251A motor driver instance.
  * @param  drv8251a: Pointer to the DRV8251A handle structure.
  * @retval HAL status (HAL_OK if initialization succeeded)
  */
HAL_StatusTypeDef DRV8251A_Init(DRV8251A_HandleTypeDef* drv8251a) 
{
    if (drv8251a == NULL || drv8251a->htim == NULL) {
        return HAL_ERROR;
    }

    // Start PWM on both channels (this prepares the hardware timers)
    if (HAL_TIM_PWM_Start(drv8251a->htim, drv8251a->channel1) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(drv8251a->htim, drv8251a->channel2) != HAL_OK) {
        return HAL_ERROR;
    }

    // Ensure motor starts in a stopped (Sleep/Coast) state
    __HAL_TIM_SET_COMPARE(drv8251a->htim, drv8251a->channel1, 0);
    __HAL_TIM_SET_COMPARE(drv8251a->htim, drv8251a->channel2, 0);

    return HAL_OK;
}

/**
  * @brief  Sets the motor speed and direction.
  * @param  drv8251a: Pointer to the DRV8251A handle structure.
  * @param  speed: Target speed from -1000 to +1000.
  *                Positive = Forward, Negative = Reverse, 0 = Coast / Sleep.
  */
void DRV8251A_SetSpeed(DRV8251A_HandleTypeDef* drv8251a, int16_t speed) 
{
    if (drv8251a == NULL || drv8251a->htim == NULL) {
        return;
    }

    // Constrain input speed to safe limits
    if (speed > 1000)  speed = 1000;
    if (speed < -1000) speed = -1000;

    if (speed == 0) 
    {
        // Sleep / Coast mode
        __HAL_TIM_SET_COMPARE(drv8251a->htim, drv8251a->channel1, 0);
        __HAL_TIM_SET_COMPARE(drv8251a->htim, drv8251a->channel2, 0);
    } 
    else if (speed > 0) 
    {
        // Forward: Channel 1 receives PWM, Channel 2 is tied Low
        uint32_t duty = ((uint32_t)speed * drv8251a->max_duty) / 1000;
        __HAL_TIM_SET_COMPARE(drv8251a->htim, drv8251a->channel2, 0);
        __HAL_TIM_SET_COMPARE(drv8251a->htim, drv8251a->channel1, duty);
    } 
    else 
    {
        // Reverse: Channel 2 receives PWM, Channel 1 is tied Low
        uint32_t duty = ((uint32_t)(-speed) * drv8251a->max_duty) / 1000;
        __HAL_TIM_SET_COMPARE(drv8251a->htim, drv8251a->channel1, 0);
        __HAL_TIM_SET_COMPARE(drv8251a->htim, drv8251a->channel2, duty);
    }
}

/**
  * @brief  Actively brakes the motor (low-side slow decay).
  * @param  drv8251a: Pointer to the DRV8251A handle structure.
  */
void DRV8251A_Brake(DRV8251A_HandleTypeDef* drv8251a) 
{
    if (drv8251a == NULL || drv8251a->htim == NULL) 
    {
        return;
    }
    // Pull both inputs HIGH (100% duty cycle) to trigger H-bridge brake state
    __HAL_TIM_SET_COMPARE(drv8251a->htim, drv8251a->channel1, drv8251a->max_duty);
    __HAL_TIM_SET_COMPARE(drv8251a->htim, drv8251a->channel2, drv8251a->max_duty);
}

/**
  * @brief  Puts the driver into ultra-low-power sleep mode (forces both IN1/IN2 Low).
  * @param  drv8251a: Pointer to the DRV8251A handle structure.
  */
void DRV8251A_Sleep(DRV8251A_HandleTypeDef* drv8251a) 
{
    if (drv8251a == NULL || drv8251a->htim == NULL) 
    {
        return;
    }
    // Pull both inputs LOW (the driver enters sleep mode after 1 ms)
    __HAL_TIM_SET_COMPARE(drv8251a->htim, drv8251a->channel1, 0);
    __HAL_TIM_SET_COMPARE(drv8251a->htim, drv8251a->channel2, 0);
}

/**
  * @brief  Checks if the driver has reported a fault on nFAULT.
  * @param  drv8251a: Pointer to the DRV8251A handle structure.
  * @retval 1 if a fault is active (nFAULT is LOW), 0 otherwise.
  */
uint8_t DRV8251A_CheckFault(DRV8251A_HandleTypeDef* drv8251a) 
{
    if (drv8251a == NULL || drv8251a->nFaultPort == NULL) 
    {
        return 0;
    }
    
    // nFAULT is active low
    if (HAL_GPIO_ReadPin(drv8251a->nFaultPort, drv8251a->nFaultPin) == GPIO_PIN_RESET) 
    {
        return 1;
    }
    return 0;
}

/**
  * @brief  Reads the real-time motor current using the ADC and the IPROPI current mirror.
  * @param  drv8251a: Pointer to the DRV8251A handle structure.
  * @retval Motor current in Amperes (returns 0.0 if ADC is not configured).
  */
double DRV8251A_ReadCurrent(DRV8251A_HandleTypeDef* drv8251a) 
{
    if (drv8251a == NULL || drv8251a->hadc == NULL || drv8251a->r_ipropi <= 0.0f) 
    {
        return 0.0f;
    }

    uint32_t adc_val = 0;
    
    // Start ADC conversion (using standard poll-method)
    HAL_ADC_Start(drv8251a->hadc);
    if (HAL_ADC_PollForConversion(drv8251a->hadc, HAL_MAX_DELAY) == HAL_OK) 
    {
        adc_val = HAL_ADC_GetValue(drv8251a->hadc);
    }
    HAL_ADC_Stop(drv8251a->hadc);

    // Assuming 12-bit ADC (4095 resolution) and configured reference voltage
    double v_ipropi = ((double)adc_val * drv8251a->vref) / 4095.0f;
    
    // Current calculation: I_MOTOR = V_IPROPI / (A_IPROPI * R_IPROPI)
    double i_motor = v_ipropi / (A_IPROPI_FACTOR * drv8251a->r_ipropi);
    
    return i_motor;
}

/**
  * @brief  Reads the motor current using a DMA circular buffer (averages samples for filtering).
  * @param  dev: Pointer to the DRV8251A handle structure.
  * @retval Motor current in Amperes (returns 0.0 if DMA buffer is not configured).
  */
double DRV8251A_ReadCurrentDMA(DRV8251A_HandleTypeDef* drv8251a);
{
    if (dev == NULL || dev->hadc == NULL || dev->adc_buffer == NULL || dev->adc_buffer_size == 0 || dev->r_ipropi <= 0.0f) 
    {
        return 0.0f;
    }
    uint32_t sum = 0;
    
    // Calculate the average of all samples in the circular buffer
    for (uint16_t i = 0; i < dev->adc_buffer_size; i++) 
    {
        sum += dev->adc_buffer[i];
    }
    
    float avg_raw = (float)sum / (float)dev->adc_buffer_size;
    
    // Convert average raw value to voltage (12-bit ADC)
    float v_ipropi = (avg_raw * dev->vref) / 4095.0f;
    
    // Convert voltage to motor current
    float i_motor = v_ipropi / (A_IPROPI_FACTOR * dev->r_ipropi);
    
    return i_motor;
}

/**
  * @brief  Calculates the hardware current limit (ITRIP) set by the VREF and RIPROPI resistor.
  * @param  drv8251a: Pointer to the DRV8251A handle structure.
  * @retval Current limit in Amperes.
  */
double DRV8251A_GetHardwareCurrentLimit(DRV8251A_HandleTypeDef* drv8251a) 
{
    if (drv8251a == NULL || drv8251a->r_ipropi <= 0.0f) {
        return 0.0f;
    }
    // Formula: I_TRIP = V_VREF / (A_IPROPI * R_IPROPI)
    return drv8251a->vref / (A_IPROPI_FACTOR * drv8251a->r_ipropi);
}