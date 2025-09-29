#include "main.h"
#include "math.h"
#include "OPT4048.h"

#define CIE_MATRIX_ROWS					  4
#define CIE_MATRIX_COLS					  4
#define ADC_MATRIX_ROWS					  1
#define ADC_MATRIX_COLS				      4


const double cieMatrix[CIE_MATRIX_ROWS][CIE_MATRIX_COLS] = {{.000234892992, -.0000189652390, .0000120811684, 0},
															{.0000407467441, .000198958202, -.0000158848115, .00215},
															{.0000928619404, -.0000169739553, .000674021520, 0},
															{0, 0, 0, 0}};

static double adcMatrix[ADC_MATRIX_ROWS][ADC_MATRIX_COLS] = {{0.0}};
static double resultMatrix[ADC_MATRIX_ROWS][CIE_MATRIX_COLS] = {{0.0}};
static uint16_t counter = 0;
static uint16_t counterChanel0 = 0;
static uint16_t counterChanel1 = 0;
static uint16_t counterChanel2 = 0;
static uint16_t counterChanel3 = 0;
/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  ina219 Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  value The data that must be written in the register
  */
void opt4048WriteRegister(Opt4048_t *opt4048, uint8_t registerAddress, uint16_t value)
{
  uint8_t address[2];
  uint8_t isDeviceReady;

  address[0] = (value >> 8) & 0xFF;
  address[1] = (value >> 0) & 0xFF;
  
  isDeviceReady = HAL_I2C_IsDeviceReady(opt4048->hi2c, (opt4048->devAddress) << 1, OPT4048_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(opt4048->hi2c, (opt4048->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY);
  }
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
uint16_t opt4048ReadRegister(Opt4048_t *opt4048, uint8_t registerAddress)
{
  uint8_t registerResponse[2];
  uint8_t isDeviceReady;

  isDeviceReady = HAL_I2C_IsDeviceReady(opt4048->hi2c, (opt4048->devAddress) << 1, OPT4048_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Read(opt4048->hi2c, (opt4048->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);
  }

  return ((registerResponse[0] << 8) | registerResponse[1]);
}

void opt4048Init(Opt4048_t *opt4048, I2C_HandleTypeDef *i2c)
{
    opt4048->hi2c = i2c;
    opt4048->devAddress = OPT4048_ADDRESS;
	uint16_t value = OPT4048_AUTO_RANGE | OPT4048_400_0_MS | OPT4048_CONTINUOUS;
	opt4048WriteRegister(opt4048, OPT4048_CONFIGURATION_REGISTER, value);
}

int32_t opt4048ReadRegister32(Opt4048_t *opt4048, uint8_t registerAddress)
{
	int32_t value = 0;
	uint8_t isDeviceReady;
	uint8_t i;

	uint8_t registerResponse[4] = {0}; //max buf size
	isDeviceReady = HAL_I2C_IsDeviceReady(opt4048->hi2c, (opt4048->devAddress) << 1, OPT4048_TRIALS, HAL_MAX_DELAY);

	if (isDeviceReady == HAL_OK)
	{
		HAL_I2C_Mem_Read(opt4048->hi2c, (opt4048->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, 4, HAL_MAX_DELAY);
	}

	for (i = 0; i < 4; i++)
	{
		value = (value << 8) | registerResponse[i];
	}

	return value;
}

uint16_t opt4048GetDeviceId(Opt4048_t *opt4048)
{
  uint16_t value = opt4048ReadRegister(opt4048, OPT4048_DEVICE_ID_REGISTER);
  return value;
}

void opt4048GetAllAdcCodes(Opt4048_t *opt4048)
{
	counter++;
	if (counter == 16) counter = 0;

	do
	{
		int32_t mantissaCh0   = 0;
		int32_t ch0           = opt4048ReadRegister32(opt4048, OPT4048_CH0_0);
		counterChanel0        = (ch0 & 0x000000F0) >> 4;

		int8_t  exponentCh0   = (ch0 & 0xF0000000) >> 28;
		int16_t resultMsbCh0  = (ch0 & 0x0FFF0000) >> 16;
		int8_t  resultLsbCh0  = (ch0 & 0x0000FF00) >> 8;

		opt4048->counterch0 = counterChanel0;

		mantissaCh0 = (resultMsbCh0 << 8) + resultLsbCh0;
		adcMatrix[0][0] = (double)(mantissaCh0 << exponentCh0);
	} while (opt4048->counterch0 != counter);

	do
	{
	int32_t mantissaCh1   = 0;
	int32_t ch1           = opt4048ReadRegister32(opt4048, OPT4048_CH1_0);
	counterChanel1       = (ch1 & 0x000000F0) >> 4;

	int8_t  exponentCh1   = (ch1 & 0xF0000000) >> 28;
	int16_t resultMsbCh1  = (ch1 & 0x0FFF0000) >> 16;
	int8_t  resultLsbCh1  = (ch1 & 0x0000FF00) >> 8;

	opt4048->counterch1 = counterChanel1;

	mantissaCh1 = (resultMsbCh1 << 8) + resultLsbCh1;
	adcMatrix[0][1] = (double)(mantissaCh1 << exponentCh1);
	} while (opt4048->counterch1 != counter);

	do
	{
	int32_t mantissaCh2   = 0;
	int32_t ch2           = opt4048ReadRegister32(opt4048, OPT4048_CH2_0);
	counterChanel2        = (ch2 & 0x000000F0) >> 4;

	int8_t  exponentCh2   = (ch2 & 0xF0000000) >> 28;
	int16_t resultMsbCh2  = (ch2 & 0x0FFF0000) >> 16;
	int8_t  resultLsbCh2  = (ch2 & 0x0000FF00) >> 8;

	opt4048->counterch2 = counterChanel2;

	mantissaCh2 = (resultMsbCh2 << 8) + resultLsbCh2;
	adcMatrix[0][2] = (double)(mantissaCh2 << exponentCh2);
	} while(opt4048->counterch2 != counter);

	do
	{
	int32_t mantissaCh3   = 0;
	int32_t ch3           = opt4048ReadRegister32(opt4048, OPT4048_CH3_0);
	counterChanel3        = (ch3 & 0x000000F0) >> 4;

	int8_t  exponentCh3   = (ch3 & 0xF0000000) >> 28;
	int16_t resultMsbCh3  = (ch3 & 0x0FFF0000) >> 16;
	int8_t  resultLsbCh3  = (ch3 & 0x0000FF00) >> 8;

	opt4048->counterch3 = counterChanel3;

	mantissaCh3 = (resultMsbCh3 << 8) + resultLsbCh3;
	adcMatrix[0][3] = (double)(mantissaCh3 << exponentCh3);
	} while (opt4048->counterch3 != counter);
}

void opt4048GetCie(Opt4048_t *opt4048)
{
	uint8_t i,j, k;
	for (i = 0; i < ADC_MATRIX_ROWS;  i++){
		for (j = 0; j < CIE_MATRIX_COLS; j++){
			resultMatrix[i][j] = 0.0;

			for (k = 0; k < CIE_MATRIX_ROWS; k++){
				resultMatrix[i][j] += adcMatrix[i][k] * cieMatrix[k][j];
			}
		}
	}
	opt4048->cieX = resultMatrix[0][0] / (resultMatrix[0][0] + resultMatrix[0][1] + resultMatrix[0][2]);
	opt4048->cieY = resultMatrix[0][1] / (resultMatrix[0][0] + resultMatrix[0][1] + resultMatrix[0][2]);
	opt4048->cieZ = resultMatrix[0][2] / (resultMatrix[0][0] + resultMatrix[0][1] + resultMatrix[0][2]);
	opt4048->lux = resultMatrix[0][3];
}


