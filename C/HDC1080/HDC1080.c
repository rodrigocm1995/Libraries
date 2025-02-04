#include "main.h"
#include "math.h"
#include "HDC1080.h"

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  hdc1080 Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  value The data that must be written in the register
  */
void HDC1080WriteRegister(Hdc1080_t *hdc1080, uint8_t registerAddress, uint16_t value)
{
  uint8_t address[2];
  uint8_t isDeviceReady;

  address[0] = (value >> 8) & 0xFF;
  address[1] = (value >> 0) & 0xFF;
  
  isDeviceReady = HAL_I2C_IsDeviceReady(hdc1080->hi2c, (hdc1080->devAddress) << 1, HDC1080_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(hdc1080->hi2c, (hdc1080->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY);
  }
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
uint16_t HDC1080ReadRegister(Hdc1080_t *hdc1080, uint8_t registerAddress)
{
  uint8_t registerResponse[2];
  uint8_t isDeviceReady;

  isDeviceReady = HAL_I2C_IsDeviceReady(hdc1080->hi2c, (hdc1080->devAddress) << 1, HDC1080_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Read(hdc1080->hi2c, (hdc1080->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);
  }

  return ((registerResponse[0] << 8) | registerResponse[1]);
}

uint8_t HDC1080DefaultInit(Hdc1080_t *hdc1080, I2C_HandleTypeDef *i2c, uint8_t devAddress)
{
  hdc1080->hi2c = i2c;
  hdc1080->devAddress = devAddress;
  uint8_t isDeviceReady;
  uint16_t configValue;

  configValue = ACTIVE_HEATER_DISABLED | HDC1080_TEMP_OR_HUMIDITY | HDC1080_BAT_VOLTAGE_G2_8 | HDC1080_TEMP_14BIT_RESOLUTION | HDC1080_HUMIDITY_14BIT_RESOLUTION;
  isDeviceReady = HAL_I2C_IsDeviceReady(i2c, devAddress << 1, HDC1080_TRIALS, HAL_MAX_DELAY);
  
  if (isDeviceReady == HAL_OK)
  {
    //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    writeRegister(hdc1080, HDC1080_CONFIGURATION_REGISTER, configValue);
    return 1;
  }

  return 0;
}

uint16_t HDC1080Init(Hdc1080_t *hdc1080, I2C_HandleTypeDef *i2c, uint8_t devAddress, HDC1080HeaterMode_t heater, HDC1080BAcquisitionMode_t mode, HDC1080BatteryStatus_t batStatus, HDC1080TempResolution_t tempRes, HDC1080HumidityResolution_t humRes)
{
  hdc1080->hi2c = i2c;
  hdc1080->devAddress = devAddress;
  uint8_t isDeviceReady;
  uint16_t configValue;

  configValue = heater | mode | batStatus | tempRes | humRes;
  isDeviceReady = HAL_I2C_IsDeviceReady(i2c, devAddress << 1, HDC1080_TRIALS, HAL_MAX_DELAY);
  
  if (isDeviceReady == HAL_OK)
  {
    //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    writeRegister(hdc1080, HDC1080_CONFIGURATION_REGISTER, configValue);
    return configValue;
  }

  return 0;
}

double HDC1080ReadTemperature(Hdc1080_t *hdc1080)
{
  uint16_t rawTemp = readRegister(hdc1080, HDC1080_TEMPERATURE_REGISTER);
  double temperature = (double)(rawTemp / pow(2,16)) * 165.0 - 140.0
  return temperature;
}

double HDC1080ReadHumidity(Hdc1080_t *hdc1080)
{
  uint16_t rawHumidity = readRegister(hdc1080, HDC1080_HUMIDITY_REGISTER);
  double humidity = (double)(rawHumidity/pow(2, 16)) * 100.0;
}
