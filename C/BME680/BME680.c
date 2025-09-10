#include "main.h"
#include "math.h"
#include "BME680.h"

#define maxRegAddress 0xEF
#define P0 1013.25L
#define COEFFLENGTH	  13
#define MSB(u16) (((u16) & 0xFF00U) >> 8)
#define LSB(u16) ((u16) & 0xFFU)

const uint8_t BME680RegSize[maxRegAddress+1] = {
  0,0,0,0,0,0,0,0, //00h - 07h
  0,0,0,0,0,0,0,0, //08h - 0Fh
  0,0,0,0,0,0,0,0, //10h - 17h
  0,0,0,0,0,1,0,3, //18h - 1Fh
  0,0,3,0,0,2,0,0, //20h - 27h
  0,0,2,0,0,0,0,0, //28h - 2Fh
  0,0,0,0,0,0,0,0, //30h - 37h
  0,0,0,0,0,0,2,2, //38h - 3Fh
  0,0,0,0,0,0,0,0, //40h - 47h
  0,0,0,0,0,0,0,0, //48h - 4Fh
  0,0,0,0,0,0,0,0, //50h - 57h
  0,0,0,0,0,0,0,0, //58h - 5Fh
  0,0,0,0,0,0,0,0, //60h - 67h
  0,0,0,0,0,0,0,0, //68h - 6Fh
  0,0,0,0,0,0,0,0, //70h - 77h
  0,0,0,0,0,0,0,0, //78h - 7Fh
  0,0,0,0,0,0,0,0, //80h - 87h
  0,0,2,0,1,0,2,0, //88h - 8Fh
  2,0,1,0,2,0,2,0, //90h - 97h
  1,1,0,0,2,0,2,0, //98h - 9Fh
  1,0,0,0,0,0,0,0, //A0h - A7h
  0,0,0,0,0,0,0,0, //A8h - AFh
  0,0,0,0,0,0,0,0, //B0h - B7h
  0,0,0,0,0,0,0,0, //B8h - BFh
  0,0,0,0,0,0,0,0, //C0h - C7h
  0,0,0,0,0,0,0,0, //C8h - CFh
  1,0,0,0,0,0,0,0, //D0h - D7h
  0,0,0,0,0,0,0,0, //D8h - DFh
  0,2,2,0,1,1,1,1, //E0h - E7h
  1,2,0,0,0,0,0,0, //E8h - EFh
};


static double tempFine = 0.0;
static double pressComp = 0.0;
static double temp_comp = 0.0;
static double hum_comp = 0.0;

void bme680WriteRegister8(Bme680_t *bme680, uint8_t registerAddress, uint8_t value)
{
  uint8_t address[1];
  address[0] = value;
  uint8_t isDeviceReady;
  
  isDeviceReady = HAL_I2C_IsDeviceReady(bme680->hi2c, (bme680->devAddress) << 1, BME680_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(bme680->hi2c, (bme680->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
  }
}

/**
  * @brief  Write a 16-bit data in blocking mode to a specific memory address
  * @param  bme680 Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  value The data that must be written in the register
  */
void bme680WriteRegister16(Bme680_t *bme680, uint8_t registerAddress, uint16_t value)
{
  uint8_t address[2];
  uint8_t isDeviceReady;

  address[0] = (value >> 8) & 0xFF;
  address[1] = (value >> 0) & 0xFF;
  
  isDeviceReady = HAL_I2C_IsDeviceReady(bme680->hi2c, (bme680->devAddress) << 1, BME680_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(bme680->hi2c, (bme680->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_16BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY);
  }
}

/**
  * @brief  Read an 8-bit data length in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */

int32_t bme680ReadRegister(Bme680_t *bme680, uint8_t registerAddress)
{
	int32_t value = 0;
	uint8_t isDeviceReady;
	uint8_t i;

	uint8_t registerResponse[3] = {0}; //max buf size
	isDeviceReady = HAL_I2C_IsDeviceReady(bme680->hi2c, (bme680->devAddress) << 1, BME680_TRIALS, HAL_MAX_DELAY);

	if (isDeviceReady == HAL_OK)
	{
		HAL_I2C_Mem_Read(bme680->hi2c, (bme680->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, BME680RegSize[registerAddress], HAL_MAX_DELAY);
	}

	for (i = 0; i < BME680RegSize[registerAddress]; i++)
	{
		value = (value << 8) | registerResponse[i];
	}

	return value;
}

/**
  * @brief  Initializes the CONFIGURATION register's device with default values 
  * @param  ina236 points to an object of the type Ina236_t 
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  devAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
void bme680Init(Bme680_t *bme680, I2C_HandleTypeDef *i2c, uint8_t devAddress)
{
  bme680->hi2c = i2c;
  bme680->devAddress = devAddress;

  uint8_t value = (uint8_t)OVERSAMPLING_X1;
  bme680WriteRegister8(bme680, CTRL_HUM_REGISTER, value);
  uint8_t os = (OVERSAMPLING_X1 << 5) | (OVERSAMPLING_X1 << 2) | (FORCED_MODE);
  bme680WriteRegister8(bme680, CTRL_MEAS_REGISTER, os);
  bme680WriteRegister8(bme680, CONFIG_REGISTER, FILTER_COEFF_127);

}

void bme680CustomInit(Bme680_t *bme680, I2C_HandleTypeDef *i2c, uint8_t devAddress, IIRFilter_t coeff)
{
  bme680->hi2c = i2c;
  bme680->devAddress = devAddress;
}


void bme680readCalCoeff(Bme680_t *bme680, bme680_cal_coeff_t *calCoeff)
{
	int32_t value = 0;
	value = bme680ReadRegister(bme680, PAR_T1);
	value = (LSB(value) << 8) | MSB(value);
	calCoeff->parT1 = (int16_t)value;

	value = bme680ReadRegister(bme680, PAR_T2);
	value = (LSB(value) << 8) | MSB(value);
	calCoeff->parT2 = (int16_t)value;

	value = bme680ReadRegister(bme680, PAR_T3);
	calCoeff->parT3 = (int8_t)value;

	value = bme680ReadRegister(bme680, TEMP_ADC) >> 4;
	calCoeff->tempAdc = value;

	value = bme680ReadRegister(bme680, PAR_P1);
	value = (LSB(value) << 8) | MSB(value);
    calCoeff->parP1 = (uint16_t)value;

    value = bme680ReadRegister(bme680, PAR_P2);
    value = (LSB(value) << 8) | MSB(value);
    calCoeff->parP2 = (uint16_t)value;

    value = bme680ReadRegister(bme680, PAR_P3);
    calCoeff->parP3 = (int8_t)value;

    value = bme680ReadRegister(bme680, PAR_P4);
    value = (LSB(value) << 8) | MSB(value);
    calCoeff->parP4 = (uint16_t)value;

    value = bme680ReadRegister(bme680, PAR_P5);
    value = (LSB(value) << 8) | MSB(value);
    calCoeff->parP5 = (uint16_t)value;

    value = bme680ReadRegister(bme680, PAR_P6);
    calCoeff->parP6 = (int8_t)value;

    value = bme680ReadRegister(bme680, PAR_P7);
    calCoeff->parP7 = (int8_t)value;

    value = bme680ReadRegister(bme680, PAR_P8);
    value = (LSB(value) << 8) | MSB(value);
    calCoeff->parP8 = (uint16_t)value;

    value = bme680ReadRegister(bme680, PAR_P9);
    value = (LSB(value) << 8) | MSB(value);
    calCoeff->parP9 = (uint16_t)value;

    value = bme680ReadRegister(bme680, PAR_P10);
    calCoeff->parP10 = (int8_t)value;

    value = bme680ReadRegister(bme680, PAR_H1);
    value = (LSB(value) << 8) | MSB(value);
    value = (MSB(value) << 4) | (LSB(value) & (0x0F));
    //value = value >> 4;
    calCoeff->parH1 = (uint16_t)value;

    value = bme680ReadRegister(bme680, PAR_H2);
    value = (MSB(value) << 8)| (LSB(value) & (0xF0));
    value = value >> 4;
    calCoeff->parH2 = (uint16_t)value;

    value = bme680ReadRegister(bme680, PAR_H3);
    calCoeff->parH3 = (int8_t)value;

    value = bme680ReadRegister(bme680, PAR_H4);
    calCoeff->parH4 = (int8_t)value;

    value = bme680ReadRegister(bme680, PAR_H5);
    calCoeff->parH5 = (int8_t)value;

    value = bme680ReadRegister(bme680, PAR_H6);
    calCoeff->parH6 = (int8_t)value;

    value = bme680ReadRegister(bme680, PAR_H7);
    calCoeff->parH7 = (int8_t)value;
}

/**
  * @brief  Read the device ID, this reading assures the correct communication with the device
  * @param  bme680 points to an object of the type bme680_t
  * @retval 8-bit data read from register's device. Must return 0x55
  */
uint8_t bme680GetId (Bme680_t *bme680){
	//uint8_t deviceId = bme680ReadRegister8(bme680, ID_REGISTER);
	uint8_t deviceId = bme680ReadRegister(bme680, ID_REGISTER);
	return deviceId;
}

void bme680EnableForcedMode(Bme680_t *bme680)
{
	uint8_t os = (OVERSAMPLING_X1 << 5) | (OVERSAMPLING_X1 << 2) | (FORCED_MODE);
	bme680WriteRegister8(bme680, CTRL_MEAS_REGISTER, os);
}


double bme680GetTemperature(Bme680_t *bme680, bme680_cal_coeff_t *calCoeff){
	int32_t tempAdc = 0;
	tempAdc = bme680ReadRegister(bme680, TEMP_ADC) >> 4;
	double var1 = (((double)tempAdc / 16384.0) - ((double)(calCoeff->parT1) / 1024.0)) * (double)(calCoeff->parT2);
	double var2 = ((((double)tempAdc / 131072.0) - ((double)(calCoeff->parT1) / 8192.0)) * (((double)tempAdc / 131072.0) - ((double)(calCoeff->parT1) / 8192.0))) * ((double)(calCoeff->parT3) * 16.0);
	tempFine = var1 + var2;
	temp_comp = tempFine / 5120.0;

	bme680EnableForcedMode(bme680);
	return temp_comp;
}

double bme680GetPressure(Bme680_t *bme680, bme680_cal_coeff_t *calCoeff)
{
	int32_t pressAdc = 0;
	pressAdc = bme680ReadRegister(bme680, PRESS_ADC) >> 4;
	double var1 = ((double)tempFine / 2.0) - 64000.0;
	double var2 = var1 * var1 * ((double)(calCoeff->parP6) / 131072.0 );
	var2 = var2 + (var1 * (double)(calCoeff->parP5) * 2.0 );
	var2 = (var2 / 4.0) + ((double)(calCoeff->parP4) * 65536.0);
	var1 = ((((double)(calCoeff->parP3) * var1 * var1) / 16384.0) + ((double)(calCoeff->parP2) * var1)) / 524288.0;
	var1 = (1.0  + (var1 / 32768.0)) * (double)(calCoeff->parP1);
	pressComp = 1048576.0 - (double)(pressAdc);
	pressComp = (( pressComp - (var2 / 4096.0)) * 6250.0) / var1;
	var1 = ((double)(calCoeff->parP9) * pressComp * pressComp) / 2147483648.0;
	var2 = pressComp * ((double)(calCoeff->parP8) / 32768.0 );
	double var3 = (pressComp / 256.0) * (pressComp / 256.0) * (pressComp / 256.0) * ((calCoeff->parP10) / 131072.0);
	pressComp = pressComp + (var1 + var2 + var3 + ((double)(calCoeff->parP7) * 128.0)) / 16.0;

	return pressComp / 100.0;
}

double bme680GetAltitude(Bme680_t *bme680)
{
	return 44330 * (1 - pow((pressComp/100)/P0,0.190294957) );
}

double bme680GetHumidity(Bme680_t *bme680, bme680_cal_coeff_t *calCoeff)
{
	uint16_t humAdc = 0;
    humAdc = bme680ReadRegister(bme680, HUM_ADC);
	double var1 = humAdc - (((double)(calCoeff->parH1) *16.0  ) + (((double)(calCoeff->parH3) / 2.0 ) * temp_comp));
	double var2 = var1 * (((double)(calCoeff->parH2) / 262144.0 ) * (1.0 + (((double)(calCoeff->parH4) / 16384.0 ) * temp_comp) + (((double)(calCoeff->parH5) / 1048576.0) * temp_comp * temp_comp)));
	double var3 = (double)(calCoeff->parH6) / 16384.0;
	double var4 = (double)(calCoeff->parH7) / 2097152.0;
	hum_comp = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);

	return  hum_comp;
}
