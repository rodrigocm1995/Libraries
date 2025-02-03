#include "main.h"
#include "math.h"
#include "BMP180.h"

/**
  * @brief  Write an 8-bit data in blocking mode to a specific memory address
  * @param  bmp180 Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  value The data that must be written in the register
  */

static long X1 = 0;
static long X2 = 0;
static long X3 = 0;
static long B3 = 0;
static long B5 = 0;
static long B6 = 0;
static unsigned long B4 = 0;
static unsigned long B7 = 0;
static long p = 0;
static long altitude = 0;
static long UP = 0;



void writeRegister8(Bmp180_t *bmp180, uint8_t registerAddress, uint8_t value)
{
  uint8_t address[1];
  address[0] = value;
  uint8_t isDeviceReady;
  
  isDeviceReady = HAL_I2C_IsDeviceReady(bmp180->hi2c, (bmp180->devAddress) << 1, BMP180_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(bmp180->hi2c, (bmp180->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
  }
}

/**
  * @brief  Write a 16-bit data in blocking mode to a specific memory address
  * @param  bmp180 Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  value The data that must be written in the register
  */
void writeRegister16(Bmp180_t *bmp180, uint8_t registerAddress, uint16_t value)
{
  uint8_t address[2];
  uint8_t isDeviceReady;

  address[0] = (value >> 8) & 0xFF;
  address[1] = (value >> 0) & 0xFF;
  
  isDeviceReady = HAL_I2C_IsDeviceReady(bmp180->hi2c, (bmp180->devAddress) << 1, BMP180_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(bmp180->hi2c, (bmp180->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_16BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY);
  }
}

/**
  * @brief  Read an 8-bit data length in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
uint8_t readRegister8(Bmp180_t *bmp180, uint8_t registerAddress)
{
  uint8_t registerResponse[1] = {0};
  uint8_t isDeviceReady;

  isDeviceReady = HAL_I2C_IsDeviceReady(bmp180->hi2c, (bmp180->devAddress) << 1, BMP180_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Read(bmp180->hi2c, (bmp180->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);
  }

  return registerResponse[0];
}

/**
  * @brief  Read an 16-bit data length in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
uint16_t readRegister16(Bmp180_t *bmp180, uint8_t registerAddress)
{
  uint8_t registerResponse[2];
  uint8_t isDeviceReady;

  isDeviceReady = HAL_I2C_IsDeviceReady(bmp180->hi2c, (bmp180->devAddress) << 1, BMP180_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Read(bmp180->hi2c, (bmp180->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);
  }

  return ((registerResponse[0] << 8) | registerResponse[1]);
}

/**
  * @brief  Read an 24-bit data length in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
uint32_t readRegister24(Bmp180_t *bmp180, uint8_t registerAddress)
{
  uint8_t registerResponse[3]= {0};
  uint8_t isDeviceReady;

  isDeviceReady = HAL_I2C_IsDeviceReady(bmp180->hi2c, (bmp180->devAddress) << 1, BMP180_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Read(bmp180->hi2c, (bmp180->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (int8_t*)registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);
  }
  uint16_t response = ((registerResponse[0] << 16) | registerResponse[1]<<8 | registerResponse[2]) >> (8 - (uint8_t)(bmp180->mode));
  return response;
}

/**
  * @brief  Initializes the CONFIGURATION register's device with default values 
  * @param  ina236 points to an object of the type Ina236_t 
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  devAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
void bmp180Init(Bmp180_t *bmp180, I2C_HandleTypeDef *i2c, uint8_t devAddress)
{
  bmp180->hi2c = i2c;
  bmp180->devAddress = devAddress;
  bmp180->mode = 0;

}

void bmp180CustomInit(Bmp180_t *bmp180, I2C_HandleTypeDef *i2c, uint8_t devAddress, AccuracyMode_t accuracyMode)
{
  bmp180->hi2c = i2c;
  bmp180->devAddress = devAddress;
  bmp180->mode = accuracyMode;

}

void readCalibrationCoefficients(Bmp180_t *bmp180, bmp180_cal_coeff_t *calCoeff){

  for (uint8_t reg = BMP180_CAL_COEFF_AC1; reg <= BMP180_CAL_COEFF_MD; reg++){
    uint32_t value = 0;
    value = readRegister16(bmp180, reg);

    switch(reg){
      case BMP180_CAL_COEFF_AC1: calCoeff->bmpAC1 = value;
      break;

      case BMP180_CAL_COEFF_AC2: calCoeff->bmpAC2 = value;
      break;

      case BMP180_CAL_COEFF_AC3: calCoeff->bmpAC3 = value;
      break;

      case BMP180_CAL_COEFF_AC4: calCoeff->bmpAC4 = value;
      break;

      case BMP180_CAL_COEFF_AC5: calCoeff->bmpAC5 = value;
      break;

      case BMP180_CAL_COEFF_AC6: calCoeff->bmpAC6 = value;
      break;

      case BMP180_CAL_COEFF_B1: calCoeff->bmpB1 = value;
      break;

      case BMP180_CAL_COEFF_B2: calCoeff->bmpB2 = value;
      break;

      case BMP180_CAL_COEFF_MB: calCoeff->bmpMB = value;
      break;

      case BMP180_CAL_COEFF_MC: calCoeff->bmpMC = value;
      break;

      case BMP180_CAL_COEFF_MD: calCoeff->bmpMD = value;
      break;
    }
    HAL_Delay(5);
  } 
}

/**
  * @brief  Read the device ID, this reading assures the correct communication with the device
  * @param  bmp180 points to an object of the type Bmp180_t
  * @retval 8-bit data read from register's device. Must return 0x55
  */
uint8_t readBmp180Id (Bmp180_t *bmp180){
	uint8_t deviceId = readRegister8(bmp180, DEVICE_ID);
	return deviceId;
}


uint16_t readUncompensatedTemperature(Bmp180_t *bmp180){
	writeRegister8(bmp180, CTRL_MEAS, 0x2E);
	HAL_Delay(5);
	uint16_t response = readRegister16(bmp180, OUT_MSB);

	return response;
}

long getTemperature(Bmp180_t *bmp180, bmp180_cal_coeff_t *calCoeff){
	uint16_t UT = readUncompensatedTemperature(bmp180);

	X1 = (UT - calCoeff->bmpAC6) * calCoeff->bmpAC5 / pow(2, 15);
	X2 = (calCoeff->bmpMC * pow(2, 11)) / (X1 + calCoeff->bmpMD);
	B5 = X1 + X2;

	long temperature = ((B5 + 8) / pow(2, 4)) / 10;

	return temperature;
}

long readUncompensatedPressure(Bmp180_t *bmp180){
	writeRegister8(bmp180, CTRL_MEAS, 0x34 + ((bmp180->mode) << 6) );
	HAL_Delay(5);
	long response = readRegister24(bmp180, OUT_MSB);

	return response;
}

long getAltitude(Bmp180_t *bmp180, bmp180_cal_coeff_t *calCoeff){

	UP = readUncompensatedPressure(bmp180);
	B6 = B5 - 4000;
	X1 = (calCoeff->bmpB2 * ((pow(B6, 2) / pow(2, 12)) )) / pow(2, 11);
	X2 = (calCoeff->bmpAC2 * B6) / pow(2, 11);
	X3 = X1 + X2;
	B3 = (((calCoeff->bmpAC1*4 + X3) << (uint8_t)(bmp180->mode)) + 2)/4;
	X1 =(calCoeff->bmpAC3 * B6) / pow(2, 13);
	X2 = (calCoeff->bmpB1 * ( pow(B6, 2) / pow(2, 12) )) / pow(2, 16);
	X3 = ((X1 + X2) + 2)/pow(2, 2);
	B4 = (calCoeff->bmpAC4) * (unsigned long)(X3 + 32768) / pow(2, 15);
	B7 = ((unsigned long)UP - B3) * (50000 >> (uint8_t)(bmp180->mode));

	if (B7 < 0x80000000){
		p = ((unsigned long)B7 * 2)/B4;
	}else{
		p = (B7/B4) * 2;
	}
	X1 = (p / pow(2, 8)) * (p/ pow(2, 8));
	X1 = (X1*3038)/pow(2, 16);
	X2 = (-7357*p)/pow(2, 16);
	p = ( p + (X1 + X2 + 3791)/(pow(2, 4)) ) / 100;

	altitude = 44330 * (1 - pow(p/STD_PRESSURE, 0.190294957) );
	return altitude;
}

