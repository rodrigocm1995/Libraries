#include "main.h"
#include "math.h"
#include "INA219.h"



/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  ina219 Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  value The data that must be written in the specified register
  */
void writeRegister(Ina219_t *ina219, uint8_t registerAddress, uint16_t value)
{
  uint8_t address[2];
  
  address[0] = (value >> 8) & 0xFF;
  address[1] = (value >> 0) & 0xFF;
  uint8_t isDeviceReady = HAL_I2C_IsDeviceReady(ina219->hi2c, (ina219->devAddress) << 1, INA219_TRIALS, HAL_MAX_DELAY);
  if (isDeviceReady == HAL_OK)
  {
	  HAL_I2C_Mem_Write(ina219->hi2c, (ina219->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY);
  }
  
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16 bit data read from register's device
  */
uint16_t readRegister(Ina219_t *ina219, uint8_t registerAddress)
{
  uint8_t registerResponse[2];
  uint8_t isDeviceReady = HAL_I2C_IsDeviceReady(ina219->hi2c, (ina219->devAddress) << 1, INA219_TRIALS, HAL_MAX_DELAY);
  if (isDeviceReady == HAL_OK)
  {
	  HAL_I2C_Mem_Read(ina219->hi2c, (ina219->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);
  }

  return ((registerResponse[0] << 8) | registerResponse[1]);
}

uint8_t ina219_default_init(Ina219_t *ina219, I2C_HandleTypeDef *i2c, uint8_t devAddress)
{
  ina219->hi2c = i2c;
  ina219->devAddress = devAddress;

  uint16_t configValue = INA219_BUSVOLTAGERANGE_32V | INA219_PGAGAIN_320_MILI_VOLT | INA219_BUS_ADC_12_BIT_RESOLUTION | INA219_SHUNT_ADC_12_BIT_RESOLUTION | INA219_SHUNTBUS_CONTINUOUS_MODE;
  uint8_t isDeviceReady = HAL_I2C_IsDeviceReady(i2c, devAddress << 1, INA219_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    writeRegister(ina219, INA219_CONFIGURATION_REG, configValue);
    return 1;
  }
  return 0;
}

/**
  * @brief  Initializing device with custom values, that value is written in CONFIGURATION REGISTER
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @param  busVoltageRange: set the maximun bus voltage supportef by device 16V or 32V range
  * @param  pgaGain: Sets PGA (programmable Gain Amplifier) gain and range, this is the maximum voltage range the device will measure between Rshunt terminals
  * @param  busAdcResolution:
  * @retval return 1 if communication is successfully
  */
uint8_t ina219_init(Ina219_t *ina219, I2C_HandleTypeDef *i2c, uint8_t devAddress, BusVoltageRange_t busvoltageRange, ShuntPGAGain_t pgaGain, BusADCResolution_t busAdcResolution, ShuntADCResolution_t shuntAdcResolution, Mode_t mode)
{
  ina219->hi2c = i2c;
  ina219->devAddress = devAddress;

  uint16_t configValue = busvoltageRange | pgaGain | busAdcResolution | shuntAdcResolution | mode;
  uint8_t isDeviceReady = HAL_I2C_IsDeviceReady(i2c, devAddress << 1, INA219_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    writeRegister(ina219, INA219_CONFIGURATION_REG, configValue);
    return 1;
  }
  return 0;
}

/**
  * @brief  Write into the CALIBRATION_REGISTER which enables the user to scale Current Register and Power Register
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  RshuntValue Target device address: The device 7 bits address value
  * @param  maxCurrent: set the maximun bus voltage supported by device 16V or 32V range
  * @retval return 1 if communication is successfully
  */
uint16_t ina219_calibration(Ina219_t *ina219, double RshuntValue, uint8_t maxCurrent)
{
	ina219->Rshunt = RshuntValue;
	ina219->Imax = maxCurrent;
	//double currentLSB = (double)((maxCurrent) / pow(2, 15)) * 2.0;
	double currentLSB = (double)((maxCurrent) / pow(2, 15));
	uint16_t cal = trunc(0.04096 / (currentLSB * RshuntValue));
	if (cal > pow(2,16))
	{
		return 0;
	}
	else
	{
		uint16_t cal1 = cal;
		//cal1 = 3428;
		writeRegister(ina219, INA219_CALIBRATION_REG, cal1);
		return cal1;
	}
}

double ina219ReadShuntVoltage(Ina219_t *ina219)
{
	double result = readRegister(ina219, INA219_SHUNTVOLTAGE_REG);

	return (result * 0.01);
}

double ina219ReadBusVoltage(Ina219_t *ina219)
{
	double result = (readRegister(ina219, INA219_BUSVOLTAGE_REG)) >> 3;
	return result * (4.0/1000.0);
}

double ina219ReadCurrent(Ina219_t *ina219)
{
	double rawCurrent = readRegister(ina219, INA219_CURRENT_REG);
	//double current = rawCurrent * ((ina219->Imax) / pow(2, 15)) * (1000.0) * (2.0); // pow(2,15)*1000*2 = CURRENT_LSB
	double current = rawCurrent * ((ina219->Imax) / pow(2, 15)) * (1000.0);
	return current;
}

double ina219ReadPower(Ina219_t *ina219)
{
	double rawPower = readRegister(ina219, INA219_POWER_REG);
	//double currentLSB = (ina219->Imax) / pow(2, 15) * 2.0;
	double currentLSB = (ina219->Imax) / pow(2, 15);
	double powerLSB = 20.0 * currentLSB;
	double power = rawPower * powerLSB;

	return power;
}

_Bool ina219DataReady(Ina219_t *ina219)
{
  _Bool isDataReady;

  uint16_t conversionReady = readRegister(ina219, INA219_BUSVOLTAGE_REG);
  isDataReady = CHECK_BIT(conversionReady,1);

  return isDataReady;
}

uint8_t ina219CorrectedCalibration(Ina219_t *ina219, uint16_t calValue, double inaCurrent, double measuredShuntCurrent)
{
	uint16_t correctedFullScaleCal = trunc((calValue*measuredShuntCurrent)/(inaCurrent));
	writeRegister(ina219, INA219_CALIBRATION_REG, correctedFullScaleCal);

	return 1;
}

