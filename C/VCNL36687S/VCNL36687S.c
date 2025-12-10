#include "main.h"
#include <math.h>
#include <VCNL36687S.h>

#define MSB(u16) (((u16) & 0xFF00U) >> 8)
#define LSB(u16) ((u16) & 0xFFU)

volatile uint16_t response = 0;



void VCNL36687_WriteRegister(VCNL36687_HandleTypeDef *vcnl36687, uint8_t registerAddress, uint16_t value)
{
  uint8_t txBuf[2]; // Buffer to store LSB and MSB bytes.
  uint8_t isDeviceReady; // Check if target device is ready for communication

  txBuf[0] = LSB(value);
  txBuf[1] = MSB(value);
  
  isDeviceReady = HAL_I2C_IsDeviceReady(vcnl36687->hi2c, (vcnl36687->devAddress) << 1, VCNL36687S_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(vcnl36687->hi2c, (vcnl36687->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)txBuf, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY);
  }
}


uint16_t VCNL36687_ReadRegister(VCNL36687_HandleTypeDef *vcnl36687, uint8_t registerAddress)
{
  uint8_t registerResponse[2]; // Buffer that stores the sensor data response
  uint8_t isDeviceReady; // Check if target device is ready for communication

  isDeviceReady = HAL_I2C_IsDeviceReady(vcnl36687->hi2c, (vcnl36687->devAddress) << 1, VCNL36687S_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Read(vcnl36687->hi2c, (vcnl36687->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);
  }

  return ((registerResponse[1] << 8) | registerResponse[0]);
}


uint16_t VCNL36687_GetDeviceId(VCNL36687_HandleTypeDef *vcnl36687)
{
  uint16_t data = 0;
  data = VCNL36687_ReadRegister(vcnl36687, VCNL36687_DEVICE_ID);
  
  return data;
}

uint16_t VCNL36687_GetConfig1_2(VCNL36687_HandleTypeDef *vcnl36687)
{
	  uint16_t data = 0;
	  data = VCNL36687_ReadRegister(vcnl36687, VCNL36687_PSCONFIG_1_2);

	  return data;
}

uint16_t VCNL36687_GetConfig3_4(VCNL36687_HandleTypeDef *vcnl36687)
{
	  uint16_t data = 0;
	  data = VCNL36687_ReadRegister(vcnl36687, VCNL36687_PSCONFIG_3_4);

	  return data;
}

void VCNL36687_SetPeriod(VCNL36687_HandleTypeDef *vcnl36687, VCNL36687_Period_t period)
{

	response = VCNL36687_GetConfig1_2(vcnl36687);

	response = (response & VCNL36687_PS_PERIOD_MASK) | period;
	VCNL36687_WriteRegister(vcnl36687, VCNL36687_PSCONFIG_1_2, response);
}

void VCNL36687_SetPersistence(VCNL36687_HandleTypeDef *vcnl36687, VCNL36687_Period_t persistence)
{
	response = VCNL36687_GetConfig1_2(vcnl36687);

	response = (response & VCNL36687_PS_PERS_MASK) | persistence;
	VCNL36687_WriteRegister(vcnl36687, VCNL36687_PSCONFIG_1_2, response);
}

void VCNL36687_SetInterrupt(VCNL36687_HandleTypeDef *vcnl36687, VCNL36687_Interrupt_t interrupt)
{
	response = VCNL36687_GetConfig1_2(vcnl36687);

	response = (response & VCNL36687_PS_INT_MASK) | interrupt;
	VCNL36687_WriteRegister(vcnl36687, VCNL36687_PSCONFIG_1_2, response);
}

void VCNL36687_SetSmartPersistance(VCNL36687_HandleTypeDef *vcnl36687, VCNL36687_SmartPersistance_t smartPersistance)
{
	response = VCNL36687_GetConfig1_2(vcnl36687);
	response = (response & VCNL36687_PS_INT_MASK) | smartPersistance;

	VCNL36687_WriteRegister(vcnl36687, VCNL36687_PSCONFIG_1_2, response);
}

void VCNL36687_SetPower(VCNL36687_HandleTypeDef *vcnl36687, VCNL36687_Power_t power)
{
	response = VCNL36687_GetConfig1_2(vcnl36687);

	response = (response & VCNL36687_POWER_MASK) | power;
	VCNL36687_WriteRegister(vcnl36687, VCNL36687_PSCONFIG_1_2, response);
}

void VCNL36687_SetIt(VCNL36687_HandleTypeDef *vcnl36687, VCNL36687_PS_IT_t it)
{
	response = VCNL36687_GetConfig1_2(vcnl36687);

	response = (response & VCNL36687_PS_IT_MASK) | it;
	VCNL36687_WriteRegister(vcnl36687, VCNL36687_PSCONFIG_1_2, response);
}

void VCNL36687_SetItTime(VCNL36687_HandleTypeDef *vcnl36687, VCNL36687_PS_IT_t itTime)
{
	response = VCNL36687_GetConfig1_2(vcnl36687);

	response = (response & VCNL36687_PS_ITB_MASK) | itTime;
	VCNL36687_WriteRegister(vcnl36687, VCNL36687_PSCONFIG_1_2, response);
}

void VCNL36687_SetVcselCurrent(VCNL36687_HandleTypeDef *vcnl36687, VCNL36687_VCSEL_t vcselCurrent)
{
	response = VCNL36687_GetConfig3_4(vcnl36687);
	response = (response & VCNL36687_VCSEL_I_MASK) | vcselCurrent;

	VCNL36687_WriteRegister(vcnl36687, VCNL36687_PSCONFIG_3_4, response);
}

void VCNL36687_SetResolution(VCNL36687_HandleTypeDef *vcnl36687, VCNL36687_Resolution_t resolution)
{
	response = VCNL36687_GetConfig3_4(vcnl36687);
	response = (response & VCNL36687_PS_HD_MASK) | resolution;

	VCNL36687_WriteRegister(vcnl36687, VCNL36687_PSCONFIG_3_4, response);
}

void VCNL36687_Init(VCNL36687_HandleTypeDef *vcnl36687, I2C_HandleTypeDef *i2c)
{
  vcnl36687->hi2c = i2c;
  vcnl36687->devAddress = VCNL36687S_ADDRESS;
  VCNL36687_SetPower(vcnl36687, VCNL36687_POWER_ON);
  VCNL36687_SetPeriod(vcnl36687, VCNL36687_PERIOD_8_MS);
  VCNL36687_SetIt(vcnl36687, VCNL36687_PS_8T);
  VCNL36687_SetVcselCurrent(vcnl36687,  VCNL36687_VCSEL_20_MA);
  VCNL36687_SetResolution(vcnl36687,VCNL36687_PS_16_BITS);
  VCNL36687_SetItTime(vcnl36687, VCNL36687_PS_ITB_50US);
}

uint16_t VCNL36687_GetData(VCNL36687_HandleTypeDef *vcnl36687)
{
	uint16_t data = VCNL36687_ReadRegister(vcnl36687, VCNL36687_PS_DATA);

	return data;

}

