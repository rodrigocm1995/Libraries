#include "main.h"
#include "math.h"
#include "OPT3001.h"

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  ina219 Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  value The data that must be written in the register
  */
void opt3001WriteRegister(Opt3001_t *opt3001, uint8_t registerAddress, uint16_t value)
{
  uint8_t address[2];
  uint8_t isDeviceReady;

  address[0] = (value >> 8) & 0xFF;
  address[1] = (value >> 0) & 0xFF;
  
  isDeviceReady = HAL_I2C_IsDeviceReady(opt3001->hi2c, (opt3001->devAddress) << 1, OPT3001_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(opt3001->hi2c, (opt3001->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY);
  }
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
uint16_t opt3001ReadRegister(Opt3001_t *opt3001, uint8_t registerAddress)
{
  uint8_t registerResponse[2];
  uint8_t isDeviceReady;

  isDeviceReady = HAL_I2C_IsDeviceReady(opt3001->hi2c, (opt3001->devAddress) << 1, OPT3001_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Read(opt3001->hi2c, (opt3001->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);
  }

  return ((registerResponse[0] << 8) | registerResponse[1]);
}

uint8_t opt3001Init(Opt3001_t *opt3001, I2C_HandleTypeDef *i2c, uint8_t devAddress, Exponent_t exponentRange, ConversionTime_t conversionTime, ModeOfConversion_t mode, LatchField_t latchField, PolarityField_t polarity, MaskExponent_t mask, FaultCountField_t faultCount)
{
  opt3001->hi2c = i2c;
  opt3001->devAddress = devAddress;
  uint8_t isDeviceReady;
  uint16_t configValue;

  configValue = exponentRange | conversionTime | mode | 0x0000 | latchField | polarity | mask | faultCount;
  isDeviceReady = HAL_I2C_IsDeviceReady(i2c, devAddress << 1, OPT3001_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    writeRegister(opt3001, OPT3001_CONFIGURATION_REGISTER, configValue);
    return 1;
  }

  return 0;
}

void opt3001SetLowLimit(Opt3001_t *opt3001, double lowLimit)
{
  uint16_t exponent;
  uint16_t tempExponent;
  uint16_t TL; // TL[11:0] These bits are the result in straight binary coding
  uint16_t result;

  if (lowLimit <= FULL_SCALE_RANGE_40_95)
  {
    exponent = OPT3001_FSR_40DOT95;
  }
  else if (lowLimit > FULL_SCALE_RANGE_40_95 && lowLimit <= FULL_SCALE_RANGE_81_90)
  {
    exponent = OPT3001_FSR_81DOT90;
  }
  else if (lowLimit > FULL_SCALE_RANGE_81_90 && lowLimit <= FULL_SCALE_RANGE_163_80)
  {
    exponent = OPT3001_FSR_163DOT80;
  }
  else if (lowLimit > FULL_SCALE_RANGE_163_80 && lowLimit <= FULL_SCALE_RANGE_327_60)
  {
    exponent = OPT3001_FSR_327DOT60;
  }
  else if (lowLimit > FULL_SCALE_RANGE_327_60 && lowLimit <= FULL_SCALE_RANGE_655_20)
  {
    exponent = OPT3001_FSR_655DOT20;
  }
  else if (lowLimit > FULL_SCALE_RANGE_655_20 && lowLimit <= FULL_SCALE_RANGE_1310_40)
  {
    exponent = OPT3001_FSR_1310DOT40;
  }
  else if (lowLimit > FULL_SCALE_RANGE_1310_40 && lowLimit <= FULL_SCALE_RANGE_2620_80)
  {
    exponent = OPT3001_FSR_2620DOT80;
  }
  else if (lowLimit > FULL_SCALE_RANGE_2620_80 && lowLimit <= FULL_SCALE_RANGE_5241_60)
  {
    exponent = OPT3001_FSR_5241DOT60;
  }
  else if (lowLimit > FULL_SCALE_RANGE_5241_60 && lowLimit <= FULL_SCALE_RANGE_10483_20)
  {
    exponent = OPT3001_FSR_10483DOT20;
  }
  else if (lowLimit > FULL_SCALE_RANGE_10483_20 && lowLimit <= FULL_SCALE_RANGE_20966_40)
  {
    exponent = OPT3001_FSR_20966DOT40;
  }
  else if (lowLimit > FULL_SCALE_RANGE_20966_40 && lowLimit <= FULL_SCALE_RANGE_41932_80)
  {
    exponent = OPT3001_FSR_41932DOT80;
  }
  else if (lowLimit > FULL_SCALE_RANGE_41932_80 && lowLimit <= FULL_SCALE_RANGE_83865_60)
  {
    exponent = OPT3001_FSR_83865DOT60;
  }

  tempExponent = exponent >> 12;
  TL = lowLimit / ((0.01) * pow(2, tempExponent));
  result = exponent | TL;

  writeRegister(opt3001, OPT3001_LOW_LIMIT_REGISTER, result);
}

double opt3001ReadLowLimit(Opt3001_t *opt3001)
{
  double luxValue;

  uint16_t result = readRegister(opt3001, OPT3001_LOW_LIMIT_REGISTER);
  luxValue = (0.01) * pow(2, (result >> 12)) * (result & 0x0FFF);

  return luxValue;
}

void opt3001SetHighLimit(Opt3001_t *opt3001, double highLimit)
{
  uint16_t exponent;
  uint16_t tempExponent;
  uint16_t TH; // TL[11:0] These bits are the result in straight binary coding
  uint16_t result;

  if (highLimit <= FULL_SCALE_RANGE_40_95)
  {
    exponent = OPT3001_FSR_40DOT95;
  }
  else if (highLimit > FULL_SCALE_RANGE_40_95 && highLimit <= FULL_SCALE_RANGE_81_90)
  {
    exponent = OPT3001_FSR_81DOT90;
  }
  else if (highLimit > FULL_SCALE_RANGE_81_90 && highLimit <= FULL_SCALE_RANGE_163_80)
  {
	exponent = OPT3001_FSR_81DOT90;
  }
  else if (highLimit > FULL_SCALE_RANGE_163_80 && highLimit <= FULL_SCALE_RANGE_327_60)
  {
    exponent = OPT3001_FSR_327DOT60;
  }
  else if (highLimit > FULL_SCALE_RANGE_327_60 && highLimit <= FULL_SCALE_RANGE_655_20)
  {
	exponent = OPT3001_FSR_655DOT20;
  }
  else if (highLimit > FULL_SCALE_RANGE_655_20 && highLimit <= FULL_SCALE_RANGE_1310_40)
  {
	exponent = OPT3001_FSR_1310DOT40;
  }
  else if (highLimit > FULL_SCALE_RANGE_1310_40 && highLimit <= FULL_SCALE_RANGE_2620_80)
  {
    exponent = OPT3001_FSR_2620DOT80;
  }
  else if (highLimit > FULL_SCALE_RANGE_2620_80 && highLimit <= FULL_SCALE_RANGE_5241_60)
  {
	exponent = OPT3001_FSR_5241DOT60;
  }
  else if (highLimit > FULL_SCALE_RANGE_5241_60 && highLimit <= FULL_SCALE_RANGE_10483_20)
  {
    exponent = OPT3001_FSR_10483DOT20;
  }
  else if (highLimit > FULL_SCALE_RANGE_10483_20 && highLimit <= FULL_SCALE_RANGE_20966_40)
  {
    exponent = OPT3001_FSR_20966DOT40;
  }
  else if (highLimit > FULL_SCALE_RANGE_20966_40 && highLimit <= FULL_SCALE_RANGE_41932_80)
  {
    exponent = OPT3001_FSR_41932DOT80;
  }
  else if (highLimit > FULL_SCALE_RANGE_41932_80 && highLimit <= FULL_SCALE_RANGE_83865_60)
  {
	exponent = OPT3001_FSR_83865DOT60;
  }

  tempExponent = exponent >> 12;
  TH = highLimit / ((0.01) * pow(2, tempExponent));
  result = exponent | TH;

  writeRegister(opt3001, OPT3001_HIGH_LIMIT_REGISTER, result);
}

double opt3001ReadHighLimit(Opt3001_t *opt3001)
{
  double luxValue;

  uint16_t result = readRegister(opt3001, OPT3001_HIGH_LIMIT_REGISTER);
  luxValue = (0.01) * pow(2, (result >> 12)) * (result & 0x0FFF);

  return luxValue;
}


uint16_t opt3001ReadManufacturerId(Opt3001_t *opt3001)
{
  uint16_t result = readRegister(opt3001, OPT3001_MANUFACTURER_ID_REGISTER);
  return result;
}

uint16_t opt3001ReadConfigurationRegister(Opt3001_t *opt3001)
{
  uint16_t result;
  result = readRegister(opt3001, OPT3001_CONFIGURATION_REGISTER);

  return result;
}

_Bool opt3001LowLimitFlag(Opt3001_t *opt3001)
{
  uint16_t result = readRegister(opt3001, OPT3001_CONFIGURATION_REGISTER);
  _Bool isLowLimitFlagTrue = CHECK_BIT(result, FLAG_LOW_FIELD_BIT);

  return isLowLimitFlagTrue;
}

_Bool opt3001HighLimitFlag(Opt3001_t *opt3001)
{
  uint16_t result = readRegister(opt3001, OPT3001_CONFIGURATION_REGISTER);
  _Bool isHighLimitFlagTrue = CHECK_BIT(result, FLAG_HIGH_FIELD_BIT);

  return isHighLimitFlagTrue;
}

double opt3001ReadLuxValue(Opt3001_t *opt3001)
{
  uint16_t result;
  double luxValue;

  result = readRegister(opt3001, OPT3001_RESULT_REGISTER);
  luxValue = (0.01) * pow(2, (result >> 12)) * (result & 0x0FFF);

  return luxValue;
}

