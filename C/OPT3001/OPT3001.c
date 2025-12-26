#include "main.h"
#include "math.h"
#include "OPT3001.h"

volatile uint16_t value;

/**
  * @brief  Write an amount of data to the sensor in blocking mode to a specific memory address
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *                the configuration information for connecting to the sensor.
  * @param  registerAddres
  * @param  MemAddress  sensor's internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  value Data to be sent
  * @retval HAL_status
  */
HAL_StatusTypeDef OPT3001_WriteRegister(OPT3001_HandleTypeDef *opt3001, uint8_t registerAddress, uint16_t value)
{
	uint8_t address[2]; // All writable registers are two bytes
	uint8_t isDeviceReady;
	
	address[0] = (value >> 8) & 0xFF;
	address[1] = (value >> 0) & 0xFF;
	
	isDeviceReady = HAL_I2C_IsDeviceReady(opt3001->hi2c, (opt3001->devAddress) << 1, OPT3001_TRIALS, 100);
	
	if (isDeviceReady == HAL_OK)
	{
		HAL_I2C_Mem_Write(opt3001->hi2c, (opt3001->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t *)address, I2C_MEMADD_SIZE_16BIT, 100);
		return HAL_OK;
	}
	else{
		return HAL_ERROR;
	}
}

/**
  * @brief  Read an amount of data from the sensor in blocking mode from a specific memory address
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *                the configuration information for connecting to the sensor.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval unsigned int 16-bit value of the register
  */
uint16_t OPT3001_ReadRegister(OPT3001_HandleTypeDef *opt3001, uint8_t registerAddress)
{
  uint8_t registerResponse[2];
  uint8_t isDeviceReady;

  isDeviceReady = HAL_I2C_IsDeviceReady(opt3001->hi2c, (opt3001->devAddress) << 1, OPT3001_TRIALS, 100);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Read(opt3001->hi2c, (opt3001->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), 100);
    return ((registerResponse[0] << 8) | registerResponse[1]);
  }
  else
  {
	  return HAL_ERROR;
  }
}

/**
  *@brief Get the current value of the RESULT register.
  *			This register contains the result of the most recent light to 
			digital conversion. This 16-bit register has two fields: a
			4-bit exponent and a 12-bit mantissa.
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data
  *(+) [15:12] = Exponent
  *(+) [11:0]  = Fractional Result
*/
uint16_t OPT3001_GetResult(OPT3001_HandleTypeDef *opt3001)
{
	uint16_t data = 0;
	data = OPT3001_ReadRegister(opt3001, OPT3001_RESULT_REG);
	return data;
}

/**
  *@brief Get the current value of the CONFIGURATION register.
  *			This register controls the major operational modes of the device.
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data
*/
uint16_t OPT3001_GetConfiguration(OPT3001_HandleTypeDef *opt3001)
{
	uint16_t data = 0;
	data = OPT3001_ReadRegister(opt3001, OPT3001_CONFIGURATION_REG);
	return data;
}

/**
  *@brief Get the current value of the LOW_LIMIT register.
  *			This register sets the lower comparison limit for the interrupt reporting
  *			mechanisms: the INT pin, the flag high field (FH), and floag low field (FL)
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data
*/
uint16_t OPT3001_GetLowLimit(OPT3001_HandleTypeDef *opt3001)
{
	uint16_t data = 0;
	data = OPT3001_ReadRegister(opt3001, OPT3001_LOW_LIMIT_REG);
	return data;
}

/**
  *@brief Get the current value of the HIGH_LIMIT register.
  *			This register sets the higher comparison limit for the interrupt reporting
  *			mechanisms: the INT pin, the flag high field (FH), and floag low field (FL)
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data
*/
uint16_t OPT3001_GetHighLimit(OPT3001_HandleTypeDef *opt3001)
{
	uint16_t data = 0;
	data = OPT3001_ReadRegister(opt3001, OPT3001_HIGH_LIMIT_REG);
	return data;
}

/**
  *@brief Get the manufacturer ID. This register is intended to help uniquely identify
  *			the device.
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data - 5449h
*/
uint16_t OPT3001_GetManufacturerId(OPT3001_HandleTypeDef *opt3001)
{
	uint16_t data = 0;
	data = OPT3001_ReadRegister(opt3001, OPT3001_MANUFACTURER_ID_REG);
	return data;
}

/**
  *@brief Get the Device ID. This register is intended to help uniquely identify
  *			the device.
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data - 3001h
*/
uint16_t OPT3001_GetDeviceId(OPT3001_HandleTypeDef *opt3001)
{
	uint16_t data = 0;
	data = OPT3001_ReadRegister(opt3001, OPT3001_DEVICE_ID_REG);
	return data;
}

/**
  *@brief Allows the user to set the range number inside the CONFIGURATION register.
  *			The range number field selects the full-scale lux range of the device.
  *			The format if this field is the same as the result register  exponent
  *			field (E[3:0]).
  *			This register controls the major operational modes of the device.
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param range: selects the full scale lux range
  *(+) OPT3001_40_95_LUX
  *(+) OPT3001_81_90_LUX  
  *(+) OPT3001_163_80_LUX 
  *(+) OPT3001_327_60_LUX 
  *(+) OPT3001_655_20_LUX 
  *(+) OPT3001_1310_40_LUX 
  *(+) OPT3001_2620_80_LUX 
  *(+) OPT3001_5241_60_LUX 
  *(+) OPT3001_10483_20_LUX 
  *(+) OPT3001_20966_40_LUX 
  *(+) OPT3001_41932_80_LUX 
  *(+) OPT3001_83865_60_LUX 
  *(+) OPT3001_AUTOMATIC_RANGE 
  *@retval	none
*/
void OPT3001_SetRangeNumber(OPT3001_HandleTypeDef *opt3001, OPT3001_RangeNumber_HandleTypeDef range)
{
	uint16_t val = OPT3001_GetConfiguration(opt3001);
	val = (val & OPT3001_RANGE_NUMBER_MASK) | range;
	OPT3001_WriteRegister(opt3001, OPT3001_CONFIGURATION_REG, val);
}

/**
  *@brief Allows the user to set conversion time field inside the CONFIGURATION register.
  *			The conversion time determines the length of the light to digital conversion
  *			process. The choices are 100 ms and 800 ms. A longer integration time allows
  * 		for a lower noise measurement.
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param convTime:
  *(+) OPT3001_CT_100_MS
  *(+) OPT3001_CT_800_MS  
  *@retval	none
*/
void OPT3001_SetConversionTime(OPT3001_HandleTypeDef *opt3001, OPT3001_ConversionTime_HandleTypeDef convTime)
{
	value = OPT3001_GetConfiguration(opt3001);
	value = (value & OPT3001_CONV_TIME_MASK) | convTime;
	OPT3001_WriteRegister(opt3001, OPT3001_CONFIGURATION_REG, value);
}

/**
  *@brief Allows the user to set the mode of conversion operation field inside the
  *			CONFIGURATION register.
  *			The mode of conversion operation fields controls whether the device is
  *			operating in continuous conversion, single-shot, or low-power shutdown
  *			mode. The default is 00b (shutdown mode), such that upon power-up, the
  *			device only consumes operational level power after appropriately programming
  *			the device.
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param mode:
  *(+) OPT3001_SHUTDOWN
  *(+) OPT3001_SINGLE_SHOT
  *(+) OPT3001_CONTINUOUS_CONVERSION
  *@retval	none
*/
void OPT3001_SetConversionMode(OPT3001_HandleTypeDef *opt3001, OPT3001_ConversionMode_HandleTypeDef mode)
{
	value = OPT3001_GetConfiguration(opt3001);
	value = (value & OPT3001_CONV_MODE_MASK) | mode;
	OPT3001_WriteRegister(opt3001, OPT3001_CONFIGURATION_REG, value);
}

/**
  *@brief Get the overflow flag field inside the CONFIGURATION register.
  *			The overflow flag field indicates when an overflow condition occurs in the data conversion
  *			process, typically because the light illuminating the device exceeds the programmed full-scale
  *			scale range of the device. Under this condition OVF is the to 1, otherwise OVF remains at 0.
  *			The field is reevaluated on every measurement.
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	boolean data
*/
_Bool OPT3001_OverflowFlag(OPT3001_HandleTypeDef *opt3001)
{
	_Bool overflowFlag;
	value = OPT3001_GetConfiguration(opt3001);
	overflowFlag = CHECK_BIT(value,8);
	return overflowFlag;
}

/**
  *@brief Get the conversion ready field inside the CONFIGURATION register.
  *			The conversion ready field indicates when a conversion completes. The field is set to 1 at the
  *			end of a conversion and is cleared (set to 0) when the configuration register is subsequently
  *			read or written with any value except one containing the shutdown mode.
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	boolean data
*/
_Bool OPT3001_ConversionReadyFlag(OPT3001_HandleTypeDef *opt3001)
{
	_Bool convReadyFlag;
	value = OPT3001_GetConfiguration(opt3001);
	convReadyFlag = CHECK_BIT(value, 7);
	return convReadyFlag;
}

/**
  *@brief Identifies if the result of a conversion is larger that a specified level of interest.
			FH is set to 1 when the result is larger than the level in the high-limit register.
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	boolean data
*/
_Bool OPT3001_HighLimitFlag(OPT3001_HandleTypeDef *opt3001)
{
	_Bool highLimitFlag;
	value = OPT3001_GetConfiguration(opt3001);
	highLimitFlag = CHECK_BIT(value, 6);
	return highLimitFlag;
}

/**
  *@brief Identifies if the result of a conversion is lower that a specified level of interest.
			FL is set to 1 when the result is larger than the level in the low-limit register.
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	boolean data
*/
_Bool OPT3001_LowLimitFlag(OPT3001_HandleTypeDef *opt3001)
{
	_Bool lowLimitFlag;
	value = OPT3001_GetConfiguration(opt3001);
	lowLimitFlag = CHECK_BIT(value, 5);
	return lowLimitFlag;
}

/**
  *@brief Allows the user to set the latch style inside the CONFIGURATION register.
  *			The mode of conversion operation fields controls whether the device is
  *			operating in continuous conversion, single-shot, or low-power shutdown
  *			mode. The default is 00b (shutdown mode), such that upon power-up, the
  *			device only consumes operational level power after appropriately programming
  *			the device.
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param latch:
  *(+) OPT3001_HYSTERESIS_STYLE
  *(+) OPT3001_WINDOW_STYLE  
  *@retval	none
*/
void OPT3001_SetLatchStyle(OPT3001_HandleTypeDef *opt3001, OPT3001_LatchStyle_HandleTypeDef latch)
{
	value = OPT3001_GetConfiguration(opt3001);
	value = (value & OPT3001_LATCH_STYLE_MASK) | latch;
	OPT3001_WriteRegister(opt3001, OPT3001_CONFIGURATION_REG, value);
}

/**
  *@brief Allows the user to set the INT polarity inside the CONFIGURATION register.
  *			The polarity field controls the polarity or active state of the INT pin.
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param pol:
  *(+) OPT3001_INT_ACTIVE_LOW
  *(+) OPT3001_INT_ACTIVE_HIGH  
  *@retval	none
*/
void OPT3001_SetIntPolarity(OPT3001_HandleTypeDef *opt3001, OPT3001_Int_Polarity_HandleTypeDef pol)
{
	value = OPT3001_GetConfiguration(opt3001);
	value = (value & OPT3001_INT_POLARITY_MASK) | pol;
	OPT3001_WriteRegister(opt3001, OPT3001_CONFIGURATION_REG, value);
}

void OPT3001_SetExponentFIeld(OPT3001_HandleTypeDef *opt3001, OPT3001_MaskExp_HandleTypeDef maskExp)
{
	value = OPT3001_GetConfiguration(opt3001);
	value = (value & OPT3001_EXP_MASK) | maskExp;
	OPT3001_WriteRegister(opt3001, OPT3001_CONFIGURATION_REG, value);
}


/**
  *@brief Allows the user to set the Fault Count inside the CONFIGURATION register.
  *			The fault count field instructs the device as to how many consecutive fault events are
  *			required to trigger the interrupt reporting mechanisms: the INT pin, the flag high field
  *			(FH), and flag low field (FL).
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param faultCount:
  *(+) OPT3001_ONE_FAULT_COUNT
  *(+) OPT3001_TWO_FAULT_COUNTS  
  *(+) OPT3001_FOUR_FAULT_COUNTS
  *(+) OPT3001_EIGHT_FAULT_COUNTS
  *@retval	none
*/
void OPT3001_SetFaultCount(OPT3001_HandleTypeDef *opt3001, OPT3001_FaultCount_HandleTypeDef faultCount)
{
	value = OPT3001_GetConfiguration(opt3001);
	value = (value & OPT3001_FAULT_COUNT_MASK) | faultCount;
	OPT3001_WriteRegister(opt3001, OPT3001_CONFIGURATION_REG, value);
}


/**
  *@brief Set the low limit lux .
  * The formula to translate the register value into lux is given by:
  * lux = 0.01 * (2^E[3:0]) x R[11:0]
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param lowLimit
  *@retval	unsigned integer 16-bit data
*/
HAL_StatusTypeDef OPT3001_SetLowLimit(OPT3001_HandleTypeDef *opt3001, double lowLimit)
{
	uint16_t exponent;
	uint16_t mantissa;
	
	if (OPT3001_GetInnerRange(opt3001, lowLimit) == HAL_OK)
	{
		exponent = ((opt3001->innerExponent) & 0xF000) >> 12;
		mantissa = lowLimit / (0.01 * pow(2, exponent));

		value = (opt3001->innerExponent) + mantissa;
		OPT3001_WriteRegister(opt3001, OPT3001_LOW_LIMIT_REG, value);
		return HAL_OK;
	}
	else
	{
		return HAL_ERROR;
	}
}

/**
  *@brief Set the high limit lux .
  * The formula to translate the register value into lux is given by:
  * lux = 0.01 * (2^E[3:0]) x R[11:0]
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param lowLimit
  *@retval	HAL_Status
*/
HAL_StatusTypeDef OPT3001_SetHighLimit(OPT3001_HandleTypeDef *opt3001, double highLimit)
{
	uint16_t exponent;
	uint16_t mantissa;
	
	if (OPT3001_GetInnerRange(opt3001, highLimit) == HAL_OK)
	{
		exponent = ((opt3001->innerExponent) & 0xF000) >> 12;
		mantissa = highLimit / (0.01 * pow(2, exponent));

		value = (opt3001->innerExponent) + mantissa;
		OPT3001_WriteRegister(opt3001, OPT3001_HIGH_LIMIT_REG, value);
		return HAL_OK;
	}
	else
	{
		return HAL_ERROR;
	}
}


/**
  *@brief Get the current value of lux perceived by the sensor.
  * The formula to translate the register value into lux is given by:
  * lux = 0.01 * (2^E[3:0]) x R[11:0]
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	double value of lux
*/
double OPT3001_GetLux(OPT3001_HandleTypeDef *opt3001)
{
	double lux;
	uint8_t exponent;
	uint16_t mantissa;
	uint16_t rawData = OPT3001_GetResult(opt3001);
	
	exponent = (rawData & 0xF000) >> 12;
	mantissa = (rawData & 0x0FFF) >> 0;
	
	lux = 0.01 * pow(2, exponent) * mantissa;
	return lux;
}

/**
  *@brief This function allows to set the exponent according to the value in lux suggested by
  *			the user.
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param limit
  *@retval	HAL_Status
*/
HAL_StatusTypeDef OPT3001_GetInnerRange(OPT3001_HandleTypeDef *opt3001, double limit)
{
		if (limit < 40.95)
	{
		opt3001->innerExponent = OPT3001_40_95_LUX;
	}
	else if ((limit > 40.95) && (limit < 81.90))
	{
		opt3001->innerExponent = OPT3001_81_90_LUX;
	}
	else if ((limit > 81.90) && (limit < 163.80))
	{
		opt3001->innerExponent = OPT3001_163_80_LUX;
	}
	else if ((limit > 163.80) && (limit < 327.60))
	{
		opt3001->innerExponent = OPT3001_327_60_LUX;
	}
	else if ((limit > 327.60) && (limit < 655.20))
	{
		opt3001->innerExponent = OPT3001_655_20_LUX;
	}
	else if ((limit > 655.20) && (limit < 1310.40))
	{
		opt3001->innerExponent = OPT3001_1310_40_LUX;
	}
	else if ((limit > 1310.40) && (limit < 2620.80))
	{
		opt3001->innerExponent = OPT3001_2620_80_LUX;
	}
	else if ((limit > 2620.80) && (limit < 5241.60))
	{
		opt3001->innerExponent = OPT3001_5241_60_LUX;
	}
	else if ((limit > 5241.60) && (limit < 10483.20))
	{
		opt3001->innerExponent = OPT3001_10483_20_LUX;
	}
	else if ((limit > 10483.20) && (limit < 20966.40))
	{
		opt3001->innerExponent = OPT3001_20966_40_LUX;
	}
	else if ((limit > 20966.40) && (limit < 41932.80))
	{
		opt3001->innerExponent = OPT3001_41932_80_LUX;
	}
	else if ((limit > 41932.80) && (limit < 83865.60))
	{
		opt3001->innerExponent = OPT3001_83865_60_LUX;
	}
	else
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

/**
  *@brief Get the current value of lux low limit by reading out the LOW_LIMIT register
  *		and converting that value in lux.
  * The formula to translate the register value into lux is given by:
  * lux = 0.01 * (2^E[3:0]) x R[11:0]
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data
*/
double OPT3001_GetLowLimitLux(OPT3001_HandleTypeDef *opt3001)
{
	double lux;
	uint8_t exponent;
	uint16_t mantissa;

	uint16_t rawData = OPT3001_GetLowLimit(opt3001);

	exponent = (rawData & 0xF000) >> 12;
	mantissa = (rawData & 0x0FFF) >> 0;

	lux = 0.01 * pow(2, exponent) * mantissa;
	return lux;
}

/**
  *@brief Get the current value of lux high limit by reading out the HIGH_LIMIT register
  *			and converting that value in lux.
  * The formula to translate the register value into lux is given by:
  * lux = 0.01 * (2^E[3:0]) x R[11:0]
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data
*/
double OPT3001_GetHighLimitLux(OPT3001_HandleTypeDef *opt3001)
{
	double lux;
	uint8_t exponent;
	uint16_t mantissa;

	uint16_t rawData = OPT3001_GetHighLimit(opt3001);

	exponent = (rawData & 0xF000) >> 12;
	mantissa = (rawData & 0x0FFF) >> 0;

	lux = 0.01 * pow(2, exponent) * mantissa;
	return lux;
}

/**
  * @brief  Create a new instance of the OPT3001 ambient light sensor setting the I²C port and slave address
  * @param  opt3001 Pointer to a MCP9808_HandleTypeDef structure that contains
  *                the configuration information for connecting to the sensor.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval none
  */
void OPT3001_Init(OPT3001_HandleTypeDef *opt3001, I2C_HandleTypeDef *i2c)
{
	opt3001->devAddress = OPT3001_ADDRESS;
	opt3001->hi2c = i2c;

	OPT3001_SetRangeNumber(opt3001, OPT3001_AUTOMATIC_RANGE);
	OPT3001_SetConversionTime(opt3001, OPT3001_CT_800_MS);
	OPT3001_SetConversionMode(opt3001, OPT3001_CONTINUOUS_CONVERSION);
	OPT3001_SetLatchStyle(opt3001, OPT3001_HYSTERESIS_STYLE);
	OPT3001_SetIntPolarity(opt3001, OPT3001_INT_ACTIVE_LOW);
	OPT3001_SetExponentFIeld(opt3001, OPT3001_MASK_EXP_ACTIVE);
}
