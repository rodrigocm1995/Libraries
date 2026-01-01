#include "main.h"
#include "math.h"
#include "INA228.h"

#define INA228_MAX_REG_ADDRESS 0x3F
#define MSB(u16) ((u16 & 0xFF00U) >> 8)
#define LSB(u16) ((u16 & 0x00FFU) >> 0)

volatile uint16_t value;
volatile _Bool valueFlag;

const uint8_t INA228RegSize[INA228_MAX_REG_ADDRESS + 1] = {
  2,2,2,2,3,3,2,3,\
  3,5,5,2,2,2,2,2,\
  2,2,0,0,0,0,0,0,\
  0,0,0,0,0,0,0,0,\
  0,0,0,0,0,0,0,0,\
  0,0,0,0,0,0,0,0,\
  0,0,0,0,0,0,0,0,\
  0,0,0,0,0,0,2,2
};

/**
  * @brief  Write an amount of data to the sensor in blocking mode to a specific memory address
  * @param  ina228 Pointer to a INA228_HandleTypeDef structure that contains the configuration information for connecting
   	   	   	   to the sensor.
  * @param  registerAddres
  * @param  MemAddress  sensor's internal memory address
  * @param  value Data to be sent
  * @retval HAL_status
  */
HAL_StatusTypeDef INA228_WriteRegister(INA228_HandleTypeDef *ina228, uint8_t registerAddress, uint16_t value)
{
	  uint8_t address[2]; // all writable registers are two bytes
	  uint8_t isDeviceReady;

	  address[0] = (value >> 8) & 0xFF; // MSB
	  address[1] = (value >> 0) & 0xFF; // LSB

	  isDeviceReady = HAL_I2C_IsDeviceReady(ina228->hi2c, (ina228->devAddress) << 1, INA228_TRIALS, HAL_MAX_DELAY);

	  if (isDeviceReady == HAL_OK)
	  {
	    HAL_I2C_Mem_Write(ina228->hi2c, (ina228->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY);
	    return HAL_OK;
	  }
	  return HAL_ERROR;
}


/**
  * @brief  Read an amount of data from the sensor in blocking mode from a specific memory address
  * @param  ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *                the configuration information for connecting to the sensor.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval dynamic data read from register's device depending on the register length.
  * 		Min. 8-bit, Max. 16-bit
  */
uint64_t INA228_ReadRegister(INA228_HandleTypeDef *ina228, uint8_t registerAddress)
{
	uint64_t value = 0;
	uint8_t isDeviceReady;
	uint8_t i;

	uint8_t registerResponse[5] = {0}; // max buf size
	isDeviceReady = HAL_I2C_IsDeviceReady(ina228->hi2c, (ina228->devAddress) << 1, INA228_TRIALS, HAL_MAX_DELAY);

	  if (isDeviceReady == HAL_OK)
	  {
		HAL_I2C_Mem_Read(ina228->hi2c, (ina228->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, INA228RegSize[registerAddress], HAL_MAX_DELAY);
	    for (i = 0; i < INA228RegSize[registerAddress]; i++)
	    {
	    	value = (value << 8) | registerResponse[i];
	    }
	    return value;
	  }
	  return HAL_ERROR;
}

/**
  *@brief Get the current value of the CONFIG register.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data:
  *(+) [15]: RST
  *0h - Normal Operation
  *1h - System Reset
  *(+) [14]: RSTACC
  *0h - Normal Operation
  *1h - Clears registers to default values for ENERGY and CHARGE
  *(+) [13-6]: CONVRDY
  *0h - 0 s
  *1h - 2 ms
  *FFh - 510 ms
  *(+) [5]: TEMPCOMP
  *0h - Shunt temperature compensation disabled
  *1h - Shunt temperature compensation enabled
  *(+) [4]: ADCRANGE
  *0h - ±163.84 mV
  *1h - ±40.96 mV
  *(+) [3-0]: RESERVED - Always read 0
*/
uint16_t INA228_GetConfig(INA228_HandleTypeDef *ina228)
{
	uint16_t data = INA228_ReadRegister(ina228, INA228_CONFIG_REG);
	return data;
}


/**
  *@brief Get the current value of the ADC_CONFIG register.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data:
  *(+) [15:12]: MODE
  *(+) [11:9]: VBUSCT
  *(+) [8-6]: VSHCT
  *(+) [5-3]: VTCT
  *(+) [2:0]: AVG
*/
uint16_t INA228_GetAdcConfig(INA228_HandleTypeDef *ina228)
{
	uint16_t data = INA228_ReadRegister(ina228, INA228_ADC_CONFIG_REG);
	return data;
}

/**
  *@brief Get the current value of the SHUNT_CAL register.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data:
  *(+) [15]: Reserved - Always read 0
  *(+) [14:0]: The register provides the device with a conversion constant value that represents
  *			shunt resistance used to calculate current value in Amperes
*/
uint16_t INA228_GetShuntCalibration(INA228_HandleTypeDef *ina228)
{
	uint16_t data = INA228_ReadRegister(ina228, INA228_SHUNT_CAL_REG);
	return data;
}

/**
  *@brief Get the current value of the SHUNT_TEMPCO register.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data:
  *(+) [15:14]: Reserved - Always read 0
  *(+) [13:0]: Temperature coefficient of the shunt temperature compensation correction. Calculated with
  *			respect to +25 °C.
  *			The full scale value of the register is 16383 ppp/°C.
  *			The 16-bit register provides a resolution of 1ppm/°C/LSB
  *			0h = 0 ppm/°C
  *			3FFFh = 16383 ppm/°C
*/
uint16_t INA228_GetShuntTemperatureCoefficient(INA228_HandleTypeDef *ina228)
{
	uint16_t data = INA228_ReadRegister(ina228, INA228_SHUNT_TEMPCOEF_REG);
	return data;
}

/**
  *@brief Get the current value of the VSHUNT register.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 24-bit data:
  *(+) [23:4]: Differential voltage measured across the shunt output. Two's complement value.
  *			Conversion Factor:
  *			312.5 nV/LSB when ADCRANGE = 0
  *			78.125 nV/LSB when ADCRANGE = 1
  *(+) [3:0]: Reserved - Always read 0
*/
uint32_t INA228_GetShuntVoltage(INA228_HandleTypeDef *ina228)
{
	uint32_t data = INA228_ReadRegister(ina228, INA228_VSHUNT_REG);
	return data;
}

/**
  *@brief Get the current value of the VBUS register.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 24-bit data:
  *(+) [23:4]: Bus voltage output. Two's complement value, however always positive.
  *			Conversion Factor: 195.3125 uV/LSB
  *(+) [3:0]: Reserved - Always read 0
*/
uint32_t INA228_GetBusVoltage(INA228_HandleTypeDef *ina228)
{
	uint32_t data = INA228_ReadRegister(ina228, INA228_VBUS_REG);
	return data;
}

/**
  *@brief Get the current value of the DIETEMP register.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data:
  *(+) [15: 0]: Internal die temperature measurement. Two's complement value.
  *			Conversion Factor: 7.8125 m°C/LSB
*/
uint16_t INA228_GetDieTemperature(INA228_HandleTypeDef *ina228)
{
	uint16_t data = INA228_ReadRegister(ina228, INA228_DIETEMP_REG);
	return data;
}

/**
  *@brief Get the current value of the CURRENT register.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 24-bit data:
  *(+) [23:4]: Calculated output current in Amperes. Two's complement value.
  *(+) [3:0]: Reserved - Always read 0
*/
int32_t INA228_GetCurrent(INA228_HandleTypeDef *ina228)
{
	int32_t data = INA228_ReadRegister(ina228, INA228_CURRENT_REG);
	return data;
}

/**
  *@brief Get the current value of the POWER register.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 24-bit data:
  *(+) [23:0]: Calculated power output. Output value in watts.
  *			Unsigned representation. positive value.
*/
uint32_t INA228_GetPower(INA228_HandleTypeDef *ina228)
{
	uint32_t data = INA228_ReadRegister(ina228, INA228_POWER_REG);
	return data;
}

/**
  *@brief Get the current value of the POWER register.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 24-bit data:
  *(+) [39:0]: Calculated energy output.
  *(+) 		Output value is in Joules. Unsigned representation. Positive value
*/
uint64_t INA228_GetEnergy(INA228_HandleTypeDef *ina228)
{
	uint64_t data = INA228_ReadRegister(ina228, INA228_ENERGY_REG);
	return data;
}

/**
  *@brief Get the current value of the CHARGE register.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 39-bit data:
  *(+) [39:0]: Calculated charge output.
  *(+) 		Output value is in Coulombs. Two's complement value
*/
uint64_t INA228_GetCharge(INA228_HandleTypeDef *ina228)
{
	uint64_t data = INA228_ReadRegister(ina228, INA228_CHARGE_REG);
	return data;
}

/**
  *@brief Get the manufacturer ID. This register is intended to help uniquely identify
  *			the device.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data:
  *(+) [15:0]: Reads back TI in ASCII - 5449h
*/
uint16_t INA228_GetManufacturerId(INA228_HandleTypeDef *ina228)
{
	uint16_t data = INA228_ReadRegister(ina228, INA228_MANUFACTURER_ID_REG);
	return data;
}


/**
  *@brief Get the device ID. This register is intended to help uniquely identify
  *			the device.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data:
  *(+) [15:4]: DIEID - Stores the device identification bits - 228h
  *(+) [3:0]: REV ID - Device Revision Identification - 1h
  *(+) 2281h
*/
uint16_t INA228_GetDeviceId(INA228_HandleTypeDef *ina228)
{
	uint16_t data = INA228_ReadRegister(ina228, INA228_DEVICE_ID_REG);
	return data;
}

uint16_t INA228_GetDiagAlert(INA228_HandleTypeDef *ina228)
{
	uint16_t data = INA228_ReadRegister(ina228, INA228_DIAG_ALERT_REG);
	return data;
}


void INA228_Reset(INA228_HandleTypeDef *ina228)
{
	INA228_WriteRegister(ina228, INA228_CONFIG_REG, INA228_RESET);
}

void INA228_EnergyChargeReset(INA228_HandleTypeDef *ina228)
{
	uint16_t result = INA228_GetConfig(ina228);
	result = (result & INA228_ENERGY_CHARGE_RESET_MASK) | INA228_RST_ACC;
	INA228_WriteRegister(ina228, INA228_CONFIG_REG, result);

}

/**
  *@brief Allows the user to set the delay bits [13:6] for initial ADC conversion of the CONFIG register across IN+ and IN- pins
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param mode:
  *(+) INA228_NO_DELAY
  *(+) INA228_2_MS_DALY
  *(+) INA228_510_MS_DELAY
  *@retval	none
*/
void INA228_SetConversionDelay(INA228_HandleTypeDef *ina228, INA228_ConvDly_HandleTypeDef delay)
{
	uint16_t result = INA228_GetConfig(ina228);
	result = (result & INA228_CONVDLY_MASK) | delay;
	INA228_WriteRegister(ina228, INA228_CONFIG_REG, result);
}

void INA228_SetTempCompensation(INA228_HandleTypeDef *ina228, INA228_TempComp_HandleTypeDef tempCo)
{
	uint16_t result = INA228_GetConfig(ina228);
	result = (result & INA228_TEMPCOMP_MASK) | tempCo;
	INA228_WriteRegister(ina228, INA228_CONFIG_REG, result);
}

/**
  *@brief Allows the user to set the shunt full scale range selection bits [4] of the CONFIG register across IN+ and IN- pins
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param mode:
  *(+) INA228_ADCRANGE_163_84_MV
  *(+) INA228_ADCRANGE_40_96_MV

  *@retval	none
*/
void INA228_SetAdcRange(INA228_HandleTypeDef *ina228, INA228_AdcRange_HandleTypeDef adcRange)
{
	if (adcRange == INA228_ADCRANGE_163_84_MV)
	{
		ina228->AdcRange = 0;
		ina228->vshuntConvFactor = INA229_VSHUNT_CONV_FACTOR_ADC_0;
	}
	else
	{
		ina228->AdcRange = 1;
		ina228->vshuntConvFactor = INA229_VSHUNT_CONV_FACTOR_ADC_1;
	}
	uint16_t result = INA228_GetConfig(ina228);
	result = (result & INA228_ADCRANGE_MASK) | adcRange;
	INA228_WriteRegister(ina228, INA228_CONFIG_REG, result);
}

/**
  *@brief Allows the user to set the MODE bits [15:12] for continuous or triggered mode on bus voltage, shunt voltage
  *			or temperature measurement of the ADC_CONFIG register.
  *			The averaging setting applies to all active outputs. When > 0h, the output registers are updated
  *			after the averaging has completed.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param mode:
  *(+) INA228_SHUTDOWN_MODE
  *(+) INA228_BUS_VOLT_ONE_SHOT
  *(+) INA228_SHUNT_VOLT_ONE_SHOT
  *(+) INA228_SHUNT_BUS_VOLT_ONE_SHOT
  *(+) INA228_TEMP_ONE_SHOT
  *(+) INA228_TEMP_BUS_VOLT_ONE_SHOT
  *(+) INA228_TEMP_SHUNT_VOLT_ONE_SHOT
  *(+) INA228_TEMP_SHUNT_BUS_VOLT_ONE_SHOT
  *(+) INA228_BUS_VOLT_CONTINUOUS
  *(+) INA228_SHUNT_VOLT_CONTINUOUS
  *(+) INA228_SHUNT_BUS_VOLT_CONTINUOUS
  *(+) INA228_TEMP_CONTINUOUS
  *(+) INA228_TEMP_VUS_VOLT_CONTINUOUS
  *(+) INA228_TEMP_SHUNT_VOLT_CONTINUOUS
  *(+) INA228_TEMP_SHUNT_BUS_VOLT_CONTINUOUS
  *@retval	none
*/
void INA228_SetMode(INA228_HandleTypeDef *ina228, INA228_Mode_HandleTypeDef mode)
{
	uint16_t result = INA228_GetAdcConfig(ina228);
	result = (result & INA228_MODE_MASK) | mode;
	INA228_WriteRegister(ina228, INA228_ADC_CONFIG_REG, result);
}

/**
  *@brief Allows the user to set the conversion time of the bus voltage measurement bits [11:9] of the ADC_CONFIG register.
  *			The averaging setting applies to all active outputs. When > 0h, the output registers are updated
  *			after the averaging has completed.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param tempConvTime:
  *(+) INA228_50_US
  *(+) INA228_84_US
  *(+) INA228_150_US
  *(+) INA228_280_US
  *(+) INA228_540_US
  *(+) INA228_1052_US
  *(+) INA228_2074_US
  *(+) INA228_4120_US
  *@retval	none
*/
void INA228_SetBusVoltageConvTime(INA228_HandleTypeDef *ina228, INA228_ConversionTime_HandleTypeDef busConvTime)
{
	uint16_t result = INA228_GetAdcConfig(ina228);
	result = (result & INA228_VBUSCT_MASK) | (busConvTime << 9);
	INA228_WriteRegister(ina228, INA228_ADC_CONFIG_REG, result);
}

/**
  *@brief Allows the user to set the conversion time of the shunt voltage measurement bits [8:6] of the ADC_CONFIG register.
  *			The averaging setting applies to all active outputs. When > 0h, the output registers are updated
  *			after the averaging has completed.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param tempConvTime:
  *(+) INA228_50_US
  *(+) INA228_84_US
  *(+) INA228_150_US
  *(+) INA228_280_US
  *(+) INA228_540_US
  *(+) INA228_1052_US
  *(+) INA228_2074_US
  *(+) INA228_4120_US
  *@retval	none
*/
void INA228_SetShuntVoltageConvTime(INA228_HandleTypeDef *ina228, INA228_ConversionTime_HandleTypeDef shuntConvTime)
{
	uint16_t result = INA228_GetAdcConfig(ina228);
	result = (result & INA228_VSHCT_MASK) | (shuntConvTime << 6);
	INA228_WriteRegister(ina228, INA228_ADC_CONFIG_REG, result);
}

/**
  *@brief Allows the user to set the conversion time of the temperature measurement bits [5:3] of the ADC_CONFIG register.
  *			The averaging setting applies to all active outputs. When > 0h, the output registers are updated
  *			after the averaging has completed.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param tempConvTime:
  *(+) INA228_50_US
  *(+) INA228_84_US
  *(+) INA228_150_US
  *(+) INA228_280_US
  *(+) INA228_540_US
  *(+) INA228_1052_US
  *(+) INA228_2074_US
  *(+) INA228_4120_US
  *@retval	none
*/
void INA228_SetTemperatureConvTime(INA228_HandleTypeDef *ina228, INA228_ConversionTime_HandleTypeDef tempConvTime)
{
	uint16_t result = INA228_GetAdcConfig(ina228);
	result = (result & INA228_VTCT_MASK) | (tempConvTime << 3);
	INA228_WriteRegister(ina228, INA228_ADC_CONFIG_REG, result);
}

/**
  *@brief Allows the user to select the ADC sample averaging count bits [2:0] of the ADC_CONFIG register.
  *			The averaging setting applies to all active outputs. When > 0h, the output registers are updated
  *			after the averaging has completed.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param avg:
  *(+) INA228_1_SAMPLE
  *(+) INA228_4_SAMPLES
  *(+) INA228_16_SAMPLES
  *(+) INA228_64_SAMPLES
  *(+) INA228_128_SAMPLES
  *(+) INA228_256_SAMPLES
  *(+) INA228_512_SAMPLES
  *(+) INA228_1024_SAMPLES
  *@retval	none
*/
void INA228_SetAverage(INA228_HandleTypeDef *ina228, INA228_Average_HandleTypeDef avg)
{
	uint16_t result = INA228_GetAdcConfig(ina228);
	result = (result & INA228_AVERAGE_MASK) | avg;
	INA228_WriteRegister(ina228, INA228_ADC_CONFIG_REG, result);
}

/**
  *@brief For the INA228 device to report current values in Ampere units, a constant conversion value must be written in
  *			in the SHUNT_CAL register that is dependent on the maximum measured current ans the shunt resistance used in
  *			the application.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data:
  *(+) [15:4]: DIEID - Stores the device identification bits - 228h
  *(+) [3:0]: REV ID - Device Revision Identification - 1h
  *(+) 2281h
*/
void INA228_SetShuntCalibration(INA228_HandleTypeDef *ina228, double shuntResistor, double maximumCurrent)
{
	uint16_t shuntCal;

	ina228->shuntResistor = shuntResistor;
	ina228->maximumCurrent = maximumCurrent;
	ina228->currentLsb = (maximumCurrent / pow(2,19));
	//ina228->currentLsb = 1e-6;

	shuntCal = INA229_SCALING_FACTOR * (ina228->currentLsb)  * (ina228->shuntResistor);

	(ina228->AdcRange == 0) ? (shuntCal = shuntCal) : (shuntCal = 4 * shuntCal);
	INA228_WriteRegister(ina228, INA228_SHUNT_CAL_REG, shuntCal);
}

/**
  *@brief Read the latest calculated current output in Amperes. Two's complement value
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	double data type
*/
double INA228_ReadCurrent(INA228_HandleTypeDef *ina228)
{
	int32_t rawCurrent = (INA228_GetCurrent(ina228)) >> 4;
	double current = (ina228->currentLsb) * rawCurrent;

	return current * 1000.0;
}

/**
  *@brief Read the latest calculated power output in Watts. Unsigned representation. Positive value
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	double data type
*/
double INA228_ReadPower(INA228_HandleTypeDef *ina228)
{
	uint32_t rawPower = INA228_GetPower(ina228);
	double power = 3.2 * (ina228->currentLsb) * rawPower;

	return power;
}

/**
  *@brief Read the latest calculated energy output in Joules. Unsigned representation. Positive value
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	double data type
*/
double INA228_ReadEnergy(INA228_HandleTypeDef *ina228)
{
	uint64_t rawEnergy = INA228_GetEnergy(ina228);
	double energy = 16.0 * 3.2 * (ina228->currentLsb) * rawEnergy;

	return energy;
}

/**
  *@brief Read the latest calculated charge output in Coulombs. Two's complement value.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	double data type
*/
double INA228_ReadCharge(INA228_HandleTypeDef *ina228)
{
	uint64_t rawCharge = INA228_GetCharge(ina228);
	double charge = (ina228->currentLsb) * rawCharge;

	return charge;
}

/**
  *@brief Read the latest shunt voltage measured across the shunt output in volts. Two's complement value.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	double data type
*/
double INA228_ReadShuntVoltage(INA228_HandleTypeDef *ina228)
{
	double shuntVoltage;
	uint32_t rawShunt = INA228_GetShuntVoltage(ina228) >> 4;
	shuntVoltage = rawShunt * (ina228->vshuntConvFactor);
	//(ina228->AdcRange == 0) ? (shuntVoltage = rawShunt * ina228->vshuntConvFactor) : (shuntVoltage = rawShunt * INA229_VSHUNT_CONV_FACTOR_ADC_1);

	return shuntVoltage;
}

/**
  *@brief Read the latest bus voltage output in volts. Two's complement value. However, always positive.
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	double data type
*/
double INA228_ReadBusVoltage(INA228_HandleTypeDef *ina228)
{
	double busVoltage;
	uint32_t rawBus = INA228_GetBusVoltage(ina228) >> 4;
	busVoltage = INA229_VBUS_CONV_FACTOR * rawBus;

	return busVoltage;
}

/**
  *@brief Read the latest die temperature measurement in °C. Two's complement value
  *@param ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	double data type
*/
double INA228_ReadDieTemperature(INA228_HandleTypeDef *ina228)
{
	double dieTemperature;
	uint16_t rawTemp = INA228_GetDieTemperature(ina228);
	dieTemperature = INA229_DIETEMP_CONV_FACTOR * rawTemp;

	return dieTemperature;
}



void INA228_SetAlertPin(INA228_HandleTypeDef *ina228, INA228_CNVR_HandleTypeDef cnvr)
{
	uint16_t result = INA228_GetDiagAlert(ina228);
	result = (result & INA228_CNVR_MASK) | cnvr;
	INA228_WriteRegister(ina228, INA228_DIAG_ALERT_REG, result);
}

void INA228_SetAlertPinPolarity(INA228_HandleTypeDef *ina228, INA228_AlertPinPol_HandleTypeDef pol)
{
	uint16_t result = INA228_GetDiagAlert(ina228);
	result = (result & INA228_APOL_MASK) | pol;
	INA228_WriteRegister(ina228, INA228_DIAG_ALERT_REG, result);
}

/**
  * @brief  Create a new instance of the INA228 current sensor setting the I²C port and slave address
  * @param  ina228 Pointer to a INA228_HandleTypeDef structure that contains
  *                the configuration information for connecting to the sensor.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval none
  */
void INA228_Init(INA228_HandleTypeDef *ina228, I2C_HandleTypeDef *i2c)
{
	ina228->devAddress = INA228_ADDRESS;
	ina228->hi2c = i2c;

	INA228_SetConversionDelay(ina228, INA228_2_MS_DALY);
	INA228_SetTempCompensation(ina228, INA228_TEMPERATURE_COMP_ENABLED);
	INA228_SetAdcRange(ina228, INA228_ADCRANGE_163_84_MV);

	INA228_SetMode(ina228, INA228_TEMP_SHUNT_BUS_CONTINUOUS);
	INA228_SetBusVoltageConvTime(ina228, INA228_4120_US);
	INA228_SetShuntVoltageConvTime(ina228, INA228_4120_US);
	INA228_SetTemperatureConvTime(ina228, INA228_540_US);
	INA228_SetAverage(ina228, INA228_512_SAMPLES);
}
