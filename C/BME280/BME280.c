#include "main.h"
#include "math.h"
#include "BME280.h"


#define maxRegAddress 0xFF
#define TEMP_COEFF_SIZE							6
#define PRES_COEFF_SIZE							18
#define HUM_COEFF_SIZE							8
#define P0 1013.25L

static int32_t temp = 0;
static uint64_t globalValues = 0;
static uint32_t tempAdc;
static uint32_t pressAdc;
static uint16_t humAdc;
static double tFine;
static double P;

static int8_t tempCoeffArr[TEMP_COEFF_SIZE]  = {0};
static int8_t pressCoeffArr[PRES_COEFF_SIZE] = {0};
static int8_t humCoeffArr[HUM_COEFF_SIZE]    = {0};

#define MSB(u16) (((u16) & 0xFF00U) >> 8)
#define LSB(u16) ((u16) & 0xFFU)

const uint8_t BME280RegSize[maxRegAddress+1] = {
  0,0,0,0,0,0,0,0, //00h - 07h
  0,0,0,0,0,0,0,0, //08h - 0Fh
  0,0,0,0,0,0,0,0, //10h - 17h
  0,0,0,0,0,0,0,0, //18h - 1Fh
  0,0,0,0,0,0,0,0, //20h - 27h
  0,0,0,0,0,0,0,0, //28h - 2Fh
  0,0,0,0,0,0,0,0, //30h - 37h
  0,0,0,0,0,0,0,0, //38h - 3Fh
  0,0,0,0,0,0,0,0, //40h - 47h
  0,0,0,0,0,0,0,0, //48h - 4Fh
  0,0,0,0,0,0,0,0, //50h - 57h
  0,0,0,0,0,0,0,0, //58h - 5Fh
  0,0,0,0,0,0,0,0, //60h - 67h
  0,0,0,0,0,0,0,0, //68h - 6Fh
  0,0,0,0,0,0,0,0, //70h - 77h
  0,0,0,0,0,0,0,0, //78h - 7Fh
  0,0,0,0,0,0,0,0, //80h - 87h
  1,1,1,1,1,1,1,1, //88h - 8Fh
  1,1,1,1,1,1,1,1, //90h - 97h
  1,1,1,1,1,1,1,1, //98h - 9Fh
  0,1,0,0,0,0,0,0, //A0h - A7h
  0,0,0,0,0,0,0,0, //A8h - AFh
  0,0,0,0,0,0,0,0, //B0h - B7h
  0,0,0,0,0,0,0,0, //B8h - BFh
  0,0,0,0,0,0,0,0, //C0h - C7h
  0,0,0,0,0,0,0,0, //C8h - CFh
  1,0,0,0,0,0,0,0, //D0h - D7h
  0,0,0,0,0,0,0,0, //D8h - DFh
  1,1,1,1,1,1,1,0, //E0h - E7h
  0,0,0,0,0,0,0,0, //E8h - EFh
  0,0,1,1,1,1,0,8, //F0h - F7h
  0,0,0,0,0,0,0,0, //F8h - FFh
};

/**
  * @brief  Write an 8-bit data in blocking mode to a specific memory address
  * @param  BME280 Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  value The data that must be written in the register
  */

void bme280WriteRegister8(Bme280_t *bme280, uint8_t registerAddress, uint8_t value)
{
  uint8_t address[1];
  address[0] = value;
  uint8_t isDeviceReady;
  
  isDeviceReady = HAL_I2C_IsDeviceReady(bme280->hi2c, (bme280->devAddress) << 1, BME280_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(bme280->hi2c, (bme280->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
  }
}


/**
  * @brief  Write a 16-bit data in blocking mode to a specific memory address
  * @param  BME280 Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  value The data that must be written in the register
  */
void bme280WriteRegister16(Bme280_t *bme280, uint8_t registerAddress, uint16_t value)
{
  uint8_t address[2];
  uint8_t isDeviceReady;

  address[0] = (value >> 8) & 0xFF;
  address[1] = (value >> 0) & 0xFF;
  
  isDeviceReady = HAL_I2C_IsDeviceReady(bme280->hi2c, (bme280->devAddress) << 1, BME280_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(bme280->hi2c, (bme280->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_16BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY);
  }
}


/**
  * @brief  Read an 8-bit data length in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
 uint8_t bme280ReadRegister8(Bme280_t *bme280, uint8_t registerAddress)
 {
   uint8_t registerResponse[1] = {0};
   uint8_t isDeviceReady;
 
   isDeviceReady = HAL_I2C_IsDeviceReady(bme280->hi2c, (bme280->devAddress) << 1, BME280_TRIALS, HAL_MAX_DELAY);
 
   if (isDeviceReady == HAL_OK)
   {
     HAL_I2C_Mem_Read(bme280->hi2c, (bme280->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);
   }
 
   return registerResponse[0];
 }

 int64_t bme280ReadRegister(Bme280_t *bme280, uint8_t registerAddress)
 {
		int64_t value = 0;
		uint8_t isDeviceReady;
		uint8_t i;

		uint8_t registerResponse[8] = {0}; //max buf size
		isDeviceReady = HAL_I2C_IsDeviceReady(bme280->hi2c, (bme280->devAddress) << 1, BME280_TRIALS, HAL_MAX_DELAY);

		if (isDeviceReady == HAL_OK)
		{
			HAL_I2C_Mem_Read(bme280->hi2c, (bme280->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, BME280RegSize[registerAddress], HAL_MAX_DELAY);
		}

		for (i = 0; i < BME280RegSize[registerAddress]; i++)
		{
			value = (value << 8) | registerResponse[i];
		}

		return value;
 }

 void bme280ResetDevice(Bme280_t *bme280)
 {
	 bme280WriteRegister8(bme280, BME280_RESET, BME280_RESET_DEVICE);
 }

/**
  * @brief  Read a 8-bit data from BME280_DEVICE_ID register to get the Device ID 
  * @param  bme280 pointer to a Bme280_t structure that contains
  *                the configuration information for the specified sensor.
  * @retval unsigned integer 8-bits. The device ID 0x60
  */
uint8_t bme280GetId(Bme280_t *bme280)
{
  uint8_t value = bme280ReadRegister8(bme280, BME280_DEVICE_ID);
  return value;
}


/**
  * @brief  Sets the temperature sampling on BME280_CTRL_MEAS_REG by writing bits [7:5]
  * @param  bme280 pointer to a Bme280_t structure that contains
  *                the configuration information for the specified sensor.
  * @param  tempSampling Target temperature sampling: a 3-bit value that must be
  * 			   shifted to the left before writing into the register.
  * @retval None
  */
void bme280SetTempSampling(Bme280_t *bme280, Bme280Samples_t tempSampling)
{
	uint8_t val = 0;
	uint8_t reg = 0;

	reg = bme280ReadRegister(bme280, BME280_CTRL_MEAS_REG);
	val = reg | ((uint8_t)tempSampling << 5);

	bme280WriteRegister8(bme280, BME280_CTRL_MEAS_REG, val);
}

/**
  * @brief  Gets the temperature sampling on BME280_CTRL_MEAS_REG by reading bits [7:5]
  * @param  bme280 pointer to a Bme280_t structure that contains
  *                the configuration information for the specified sensor.
  * @retval unsigned integer 8-bits. The over-sampling temperature value.
  */
uint8_t	bme280GetTempSampling(Bme280_t *bme280)
{
	uint8_t value = bme280ReadRegister(bme280, BME280_CTRL_MEAS_REG);
	value = value & BME280_OVERSAMPLE_TEMP_MASK;

	return value;
}

/**
  * @brief  Sets the pressure sampling on BME280_CTRL_MEAS_REG by writing bits [4:2]
  * @param  bme280 pointer to a Bme280_t structure that contains
  *                the configuration information for the specified sensor.
  * @param  tempSampling Target pressure sampling: a 3-bit value that must be
  * 			   shifted to the left before writing into the register.
  * @retval None
  */
void bme280SetPressSampling(Bme280_t *bme280, Bme280Samples_t pressSampling)
{
	uint8_t val = 0;
	uint8_t reg = 0;

	reg = bme280ReadRegister(bme280, BME280_CTRL_MEAS_REG);
	val = reg | ((uint8_t)pressSampling << 2);

	bme280WriteRegister8(bme280, BME280_CTRL_MEAS_REG, val);
}

/**
  * @brief  Gets the pressure sampling on BME280_CTRL_MEAS_REG by reading bits [4:2]
  * @param  bme280 pointer to a Bme280_t structure that contains
  *                the configuration information for the specified sensor.
  * @retval unsigned integer 8-bits. The over-sampling pressure value.
  */
uint8_t bme280GetPressSampling(Bme280_t *bme280)
{
	uint8_t value = bme280ReadRegister(bme280, BME280_CTRL_MEAS_REG);
	value = value & BME280_OVERSAMPLE_PRES_MASK;

	return value;
}

/**
  * @brief  Sets the humidity sampling on BME280_CTRL_HUM by writing bits [2:0]
  * @param  bme280 pointer to a Bme280_t structure that contains
  *                the configuration information for the specified sensor.
  * @param  humSampling Target pressure sampling: a 3-bit value that must be
  * 				written in bits [2:0]
  * @retval None
  */
void bme280SetHumSampling(Bme280_t *bme280, Bme280Samples_t humSampling)
{
	bme280WriteRegister8(bme280, BME280_CTRL_HUM, (uint8_t)humSampling);
}

/**
  * @brief  Gets the humidity sampling on BME280_CTRL_HUM by reading bits [2:0]
  * @param  bme280 pointer to a Bme280_t structure that contains
  *                the configuration information for the specified sensor.
  * @retval unsigned integer 8-bits. The over-sampling humidity value.
  */
uint8_t bme280GetHumSampling(Bme280_t *bme280)
{
	return bme280ReadRegister(bme280, BME280_CTRL_HUM);
}

/**
  * @brief  Sets one of three different power modes. These can be selected using
  * 			   bits [1:0] in BME280_CTRL_MEAS_REG.
  * @param  bme280 pointer to a Bme280_t structure that contains
  *                the configuration information for the specified sensor.
  * @param  humSampling Target pressure sampling: a 3-bit value that must be
  * 				written in bits [2:0]
  * @retval None
  */
void bme280SetMode(Bme280_t *bme280, Bme280Mode_t mode)
{
	uint8_t val = 0;
	uint8_t reg = 0;

	reg = bme280ReadRegister(bme280, BME280_CTRL_MEAS_REG);
	val = reg | (uint8_t)mode;
	bme280WriteRegister8(bme280, BME280_CTRL_MEAS_REG, val);
}

/**
  * @brief  Sets the (inactive) standby period after a temperature, pressure and
  * 			   humidity measurement by writing bits [7:5] into the BME280_CONFIG_REG
  * @param  bme280 pointer to a Bme280_t structure that contains
  *                the configuration information for the specified sensor.
  * @param  sbTime Target standby period: a 3-bit value that must be shifted to the
  * 			   left before writing into the register.
  * @retval None
  */
void bme280SetStByTime(Bme280_t *bme280, Bme280Stby_t sbTime)
{
	uint8_t reg = 0;
	uint8_t val = 0;

	reg = bme280ReadRegister(bme280, BME280_CONFIG_REG);
	val = reg | (uint8_t)sbTime;

	bme280WriteRegister8(bme280, BME280_CONFIG_REG, val);
}

/**
  * @brief  Sets the filter coefficients, which slows down the response to the sensor inputs.
  * 			   Humidity does not require low pass filtering.
  * 			   humidity measurement by writing bits [7:5] into the BME280_CONFIG_REG
  * @param  bme280 pointer to a Bme280_t structure that contains
  *                the configuration information for the specified sensor.
  *	@param  fCoeff Target coefficient value: Number of samples generated
  * @retval None
  */
void bme280SetFilterCoeff(Bme280_t *bme280, Bme280FilterCoeff_t fCoeff)
{
	uint8_t reg = 0;
	uint8_t val = 0;

	reg = bme280ReadRegister(bme280, BME280_CONFIG_REG);
	val = reg | (uint8_t)fCoeff;

	bme280WriteRegister8(bme280, BME280_CONFIG_REG, val);

}

/**
  * @brief  Read a 8-bit data from BME280_CONFIG_REG register
  * @param  bme280 pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval unsigned integer 8-bit. The content of the BME280_CONFIG_REG register.
  */
uint8_t	bme280GetConfig(Bme280_t *bme280)
{
	uint8_t value = bme280ReadRegister(bme280, BME280_CONFIG_REG);

	return value;
}

/**
  * @brief  Read a 8-bit data from BME280_CTRL_MEAS_REG register
  * @param  bme280 pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval unsigned integer 8-bit. The content of the BME280_CTRL_MEAS_REG register.
  */
uint8_t	bme280GetCtrlMeas(Bme280_t *bme280)
{
	uint8_t value = bme280ReadRegister(bme280, BME280_CTRL_MEAS_REG);

	return value;
}


void bme280GetCalCoeff(Bme280_t *bme280)
{
	uint8_t i   = 0;
	uint8_t reg = 0;
	int8_t value = 0;
	for (reg = BME280_CAL_COEFF_T1; reg < BME280_CAL_COEFF_P1; reg++)
	{
		value = bme280ReadRegister(bme280, reg);
		tempCoeffArr[i++] = value;
	}

	bme280->digT1 = (tempCoeffArr[1] << 8) | tempCoeffArr[0];
	bme280->digT2 = (tempCoeffArr[3] << 8) | tempCoeffArr[2];
	bme280->digT3 = (tempCoeffArr[5] << 8) | tempCoeffArr[4];

	i = 0;
	for (reg = BME280_CAL_COEFF_P1; reg < BME280_CAL_COEFF_H1; reg++)
	{
		value = bme280ReadRegister(bme280, reg);
		pressCoeffArr[i++] = value;
	}

	bme280->digP1 = (uint16_t)(pressCoeffArr[1] << 8) | (uint8_t)(pressCoeffArr[0]);
	bme280->digP2 = (pressCoeffArr[3] << 8) | pressCoeffArr[2];
	bme280->digP3 = (pressCoeffArr[5] << 8) | pressCoeffArr[4];
	bme280->digP4 = (pressCoeffArr[7] << 8) | pressCoeffArr[6];
	bme280->digP5 = (pressCoeffArr[9] << 8) | pressCoeffArr[8];
	bme280->digP6 = (pressCoeffArr[11] << 8) | pressCoeffArr[10];
	bme280->digP7 = (pressCoeffArr[13] << 8) | pressCoeffArr[12];
	bme280->digP8 = (pressCoeffArr[15] << 8) | pressCoeffArr[14];
	bme280->digP9 = (pressCoeffArr[17] << 8) | pressCoeffArr[16];

	i = 1;
	for (reg = BME280_CAL_COEFF_H2; reg < 0xE8; reg++)
	{
		value = bme280ReadRegister(bme280, reg);
		humCoeffArr[i++] = value;
	}

	humCoeffArr[0] = bme280ReadRegister(bme280, BME280_CAL_COEFF_H1);
	bme280->digH1 = humCoeffArr[0];
	bme280->digH2 = (humCoeffArr[2] << 8) | humCoeffArr[1];
	bme280->digH3 = humCoeffArr[3];
	bme280->digH4 = (humCoeffArr[4] << 8) | (humCoeffArr[5] & 0x0F);
	bme280->digH5 = (humCoeffArr[6] << 8) | (humCoeffArr[5] & 0x0F);
	bme280->digH6 = humCoeffArr[7];

}

void bme280Init(Bme280_t *bme280, I2C_HandleTypeDef *i2c)
{

	bme280->devAddress = BME280_ADDRESS;
	bme280->hi2c = i2c;

	bme280SetTempSampling(bme280, BME280_SAMPLES_X1);
	bme280SetPressSampling(bme280, BME280_SAMPLES_X1);
	bme280SetMode(bme280, BME280_NORMAL_MODE);
	bme280SetHumSampling(bme280, BME280_SAMPLES_X1);
	bme280SetStByTime(bme280, BME280_STBY_10_0_MS);
	bme280SetFilterCoeff(bme280, BME280_FILTER_COEFF_8);
}

int64_t bme280GetOutput(Bme280_t *bme280)
{
	int64_t value;
	uint8_t isDeviceReady;
	uint8_t registerResponse[8] = {0};

	isDeviceReady = HAL_I2C_IsDeviceReady(bme280->hi2c, (bme280->devAddress) << 1, BME280_TRIALS, HAL_MAX_DELAY);

	if (isDeviceReady == HAL_OK)
	{
		HAL_I2C_Mem_Read(bme280->hi2c, (bme280->devAddress) << 1, 0xF7, I2C_MEMADD_SIZE_8BIT, registerResponse, 8, HAL_MAX_DELAY);
	}

	for (uint8_t i = 0; i < 8; i++)
	{
		value = (value << 8) | registerResponse[i];
	}

	return value;
}

_Bool bme280CheckDataReady(Bme280_t *bme280)
{
	_Bool isDataReady;
	uint8_t conversionReady = bme280ReadRegister(bme280, BME280_STATUS_REG);
	isDataReady = CHECK_BIT(conversionReady, 3);

	return isDataReady;
}

double bme280GetTemperature(Bme280_t *bme280)
{
	double var1, var2, T;
	//uint64_t tempAdc;

	bme280SetMode(bme280, BME280_NORMAL_MODE);
	globalValues = bme280GetOutput(bme280);

	tempAdc = (globalValues & BME2280_TEMP_ADC_MASK) >> 20;
	pressAdc = (globalValues & BME2280_PRES_ADC_MASK) >> 44;

	var1 = (double)((tempAdc / 8.0) - ((double)(bme280->digT1) * 2));
	var1 = (var1 * ((double)(bme280->digT2))) / 2048.0;
	var2 = (double)((tempAdc / 16.0) - ((double)(bme280->digT1)));
	var2 = (((var2 * var2) / 4096.0) * ((double)(bme280->digT3))) / 16384.0;

	tFine =  var1 + var2;
	T = (tFine * 5.0 + 128.0) / 256.0;

	return (double)(T / 100.0);
}

double bme280GetPressure(Bme280_t *bme280)
{
	double var1, var2, var3, var4;

	var1 = tFine - 128000.0;
	var2 = var1 * var1 * (double)(bme280->digP6);
	var2 = var2  + ((var1 * (double)(bme280->digP5)) * 131072.0);
	var2 = var2 + (((double)(bme280->digP4)) * 34359738368.0);
	var1 = ((var1 * var1 * (double)(bme280->digP3)) / 256.0) + ((var1 * ((double)(bme280->digP2)) * 4096.0));
	var3 = ((double)1) * 140737488355328.0;
	var1 = (var3 + var1) * ((double)(bme280->digP1)) / 8589934592.0;

	 if (var1 == 0) {
	    return 0; // avoid exception caused by division by zero
	  }

	  var4 = 1048576.0 - (double)pressAdc;
	  var4 = (((var4 * 2147483648.0) - var2) * 3125.0) / var1;
	  var1 = (((double)(bme280->digP9)) * (var4 / 8192.0) * (var4 / 8192.0)) / 33554432.0;
	  var2 = (((double)(bme280->digP8)) * var4) / 524288.0;
	  var4 = ((var4 + var1 + var2) / 256.0) + (((double)(bme280->digP7)) * 16.0);

	  P = (var4 / 25600.0);

	  return P;
}


double bme280GetAltitude(Bme280_t *bme280)
{
	return (44330.0 * (1.0 - pow(P/P0, 0.190294957) ));
}

