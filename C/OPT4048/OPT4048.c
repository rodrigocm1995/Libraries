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
  * @brief  Write a 16-bit value to a specific register of the OPT4048 device
  * @param  opt4048 Pointer to a OPT4048_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified OPT4048.
  * @param  registerAddress Internal register address of the OPT4048 to write to 
  * @param  value The 16-bit data word to be written into the target register
  * @retval HAL_OK: Write operation completed successfully
  * @retval HAL_ERROR: Device is not ready or write operation failed
  */
HAL_StatusTypeDef OPT4048_WriteRegister(OPT4048_HandleTypeDef *opt4048, uint8_t registerAddress, uint16_t value)
{
    uint8_t address[2];
    HAL_StatusTypeDef isDeviceReady;
    // Split the 16-bit value into two bytes (MSB first)
    address[0] = (value >> 8) & 0xFF;
    address[1] = (value >> 0) & 0xFF;
  
    // Check if the device is ready on the I2C bus
    isDeviceReady = HAL_I2C_IsDeviceReady(opt4048->hi2c, (opt4048->_devAddress) << 1, OPT4048_TRIALS, HAL_MAX_DELAY);
    if (isDeviceReady == HAL_OK)
    {
        // Write the 16-bit register to the device
        if (HAL_I2C_Mem_Write(opt4048->hi2c, (opt4048->_devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, sizeof(address), HAL_MAX_DELAY) == HAL_OK)
        {
        return HAL_OK;
        }
    }
    
    return HAL_ERROR;
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
uint16_t OPT4048_ReadRegister(OPT4048_HandleTypeDef *opt4048, uint8_t registerAddress)
{
  uint8_t registerResponse[2];
  uint8_t isDeviceReady;

  isDeviceReady = HAL_I2C_IsDeviceReady(opt4048->hi2c, (opt4048->_devAddress) << 1, OPT4048_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Read(opt4048->hi2c, (opt4048->_devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);
  }

  return ((registerResponse[0] << 8) | registerResponse[1]);
}

void OPT4048_Init(OPT4048_HandleTypeDef *opt4048, I2C_HandleTypeDef *i2c, uint8_t devAddress)
{
    opt4048->hi2c = i2c;
    opt4048->_devAddress = devAddress;

    OPT4048_SetFaultCount(opt4048, OPT4048_TWO_FAULT_COUNT);
    OPT4048_SetAlertPinPolarity(opt4048, OPT4048_ALERT_ACTIVE_HIGH);
    OPT4048_SetMode(opt4048, OPT4048_CONTINUOUS_MODE);
    OPT4048_SetConvTime(opt4048, OPT4048_200_MS);
    OPT4048_SetRange(opt4048, OPT4048_AUTOMATIC_RANGE);
    OPT4048_SetI2CType(opt4048, OPT4048_I2C_BURST_ENABLED);
}

int32_t opt4048ReadRegister32(OPT4048_HandleTypeDef *opt4048, uint8_t registerAddress)
{
	int32_t value = 0;
	uint8_t isDeviceReady;
	uint8_t i;

	uint8_t registerResponse[10] = {0}; //max buf size
	isDeviceReady = HAL_I2C_IsDeviceReady(opt4048->hi2c, (opt4048->_devAddress) << 1, OPT4048_TRIALS, HAL_MAX_DELAY);

	if (isDeviceReady == HAL_OK)
	{
		HAL_I2C_Mem_Read(opt4048->hi2c, (opt4048->_devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, 4, HAL_MAX_DELAY);
	}

	for (i = 0; i < 4; i++)
	{
		value = (value << 8) | registerResponse[i];
	}

	return value;
}

uint64_t OPT4048_ReadADCRawValues(OPT4048_HandleTypeDef *opt4048)
{
    uint64_t value = 0;
    uint8_t registerResponse[8] = {0};

    HAL_StatusTypeDef isDeviceReady = HAL_I2C_IsDeviceReady(opt4048->hi2c, (opt4048->_devAddress) << 1, OPT4048_TRIALS, HAL_MAX_DELAY);

    if (isDeviceReady)
    {
        if (HAL_I2C_Master_Receive(opt4048->hi2c, (opt4048->_devAddress) << 1, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY) != HAL_OK)
        {
            return 0; 
        }
        
        for (uint8_t i = 0; i < sizeof(registerResponse); i++)
        {
            value = (value << 8) | registerResponse[i];
        }
    }
}

/**
  * @brief  Retrieve the Device ID from the OPT4048 device
  * @param  ina236 Pointer to a OPT4048_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified OPT4048.
  * @return 16-bit Device ID (Expected value is 0x0821), or 0xFFFF if reading fails
  */
uint16_t OPT4048_GetDeviceID(OPT4048_HandleTypeDef *opt4048)
{
    // Read the Manufacturer ID register (Address: 0x11)
    uint16_t regValue = OPT4048_ReadRegister(opt4048, OPT4048_DEVICE_ID_REG);
  
    return regValue;
}

/**
  * @brief  Configure the fault count threshold for the alert mechanism
  * @details Sets the number of consecutive fault measurements required before the 
  *          alert pin is triggered. This prevents false alarms due to optical transients.
  * @param  opt4048 Pointer to a OPT4048_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified OPT4048.
  * @param  faultCount Number of consecutive faults:
  *                    @arg OPT4048_ONE_FAULT_COUNT   (0x0U): Trigger alert on 1 fault
  *                    @arg OPT4048_TWO_FAULT_COUNT   (0x1U): Trigger alert on 2 consecutive faults
  *                    @arg OPT4048_FOUR_FAULT_COUNT  (0x2U): Trigger alert on 4 consecutive faults
  *                    @arg OPT4048_EIGHT_FAULT_COUNT (0x3U): Trigger alert on 8 consecutive faults
  * @retval None
  */
void OPT4048_SetFaultCount(OPT4048_HandleTypeDef *opt4048, OPT4048_FaultCount_TypeDef faultCount)
{
    // Read the current CONFIGURATION register (Address: 0x0A)
    uint16_t regValue = OPT4048_ReadRegister(opt4048, OPT4048_CONFIGURATION_REG);

    // Clear the FAULT_COUNT bits using the mask (bits 0 and 1)
    regValue &= ~OPT4048_FAULT_COUNT_Mask;

    // Set the new mode by shifting it to its position (FAULT_COUNT_Pos = 0)
    // and protecting it with the mask
    regValue |= (faultCount << OPT4048_FAULT_COUNT_Pos) & OPT4048_FAULT_COUNT_Mask;

    // Write the updated value back to the CONFIGURATION register
    OPT4048_WriteRegister(opt4048, OPT4048_CONFIGURATION_REG, regValue);
}

/**
  * @brief  Configure the electrical polarity of the alert/interrupt pin
  * @details Sets whether the INT pin is active low or active high when an interrupt triggers.
  * @param  opt4048 Pointer to a OPT4048_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified OPT4048.
  * @param  polarity Target polarity state:
  *                  @arg OPT4048_ALERT_ACTIVE_LOW  (0x0U): Alert pin is active low (default)
  *                  @arg OPT4048_ALERT_ACTIVE_HIGH (0x1U): Alert pin is active high
  * @retval None
  */
void OPT4048_SetAlertPinPolarity(OPT4048_HandleTypeDef *opt4048, OPT4048_AlertPinPol_TypeDef polarity)
{
    // Read the current CONFIGURATION register
    uint16_t regValue = OPT4048_ReadRegister(opt4048, OPT4048_CONFIGURATION_REG);

    // Clear the INT_POL bit using the mask (bit 2)
    regValue &= ~OPT4048_INT_POL_Mask;

    // Set the new INT_POL by shifting it to its position (INT_POL_Pos = 2)
    // and protecting it with the mask
    regValue |= (polarity << OPT4048_INT_POL_Pos) & OPT4048_INT_POL_Mask;

    //Write the resulting value back to the register
    OPT4048_WriteRegister(opt4048, OPT4048_CONFIGURATION_REG, regValue);
}

/**
  * @brief  Set the operating mode of the OPT4048 device
  * @details Configures the measurement execution mode in the CONFIGURATION register.
  *          This controls whether the device is in low-power standby, executing single measurements,
  *          or performing continuous acquisitions.
  * @param  opt4048 Pointer to a OPT4048_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified OPT4048.
  * @param  mode Target operating mode from OPT4048_Mode_TypeDef:
  *              @arg OPT4048_SHUTDOWN_MODE             (0x0U): Device enters ultra-low power standby mode.
  *              @arg OPT4048_FORCED_AUTORANGE_ONE_SHOT (0x1U): Triggers a single measurement with forced automatic range selection.
  *              @arg OPT4048_ONE_SHOT_MODE             (0x2U): Triggers a single measurement using the manually configured range.
  *              @arg OPT4048_CONTINUOUS_MODE           (0x3U): Device performs measurements continuously at the set conversion rate.
  * @retval None
  */
void OPT4048_SetMode(OPT4048_HandleTypeDef *opt4048, OPT4048_Mode_TypeDef mode)
{
    // Read the current CONFIGURATION register
    uint16_t regValue = OPT4048_ReadRegister(opt4048, OPT4048_CONFIGURATION_REG);

    // Clear the OPERATING_MODE bit using the mask (bits 4 and 5)
    regValue &= ~OPT4048_OPERATING_MODE_Mask;

    // Set the new OPERATING_MODE by shifting it to its position (OPERATING_MODE_Pos = 4)
    // and protecting it with the mask
    regValue |= (mode << OPT4048_OPERATING_MODE_Pos) & OPT4048_OPERATING_MODE_Mask;

    //Write the resulting value back to the register
    OPT4048_WriteRegister(opt4048, OPT4048_CONFIGURATION_REG, regValue);
}

/**
  * @brief  Configure the conversion time for a single measurement channel
  * @details Sets the integration time for a single channel. Since the OPT4048 is a 4-channel
  *          device, the actual time to complete a full measurement cycle across all channels
  *          is 4 times the value configured here.
  * @param  opt4048 Pointer to a OPT4048_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified OPT4048.
  * @param  convTime Conversion time selection for a single channel:
  *                  @arg OPT4048_600_US  (0x0U): 600 us (Total cycle: 2.4 ms)
  *                  @arg OPT4048_1_MS    (0x1U): 1.0 ms (Total cycle: 4.0 ms)
  *                  @arg OPT4048_1_8_MS  (0x2U): 1.8 ms (Total cycle: 7.2 ms)
  *                  @arg OPT4048_3_4_MS  (0x3U): 3.4 ms (Total cycle: 13.6 ms)
  *                  @arg OPT4048_6_5_MS  (0x4U): 6.5 ms (Total cycle: 26.0 ms)
  *                  @arg OPT4048_12_7_MS (0x5U): 12.7 ms (Total cycle: 50.8 ms)
  *                  @arg OPT4048_25_MS   (0x6U): 25.0 ms (Total cycle: 100.0 ms)
  *                  @arg OPT4048_50_MS   (0x7U): 50.0 ms (Total cycle: 200.0 ms)
  *                  @arg OPT4048_100_MS  (0x8U): 100.0 ms (Total cycle: 400.0 ms)
  *                  @arg OPT4048_200_MS  (0x9U): 200.0 ms (Total cycle: 800.0 ms)
  *                  @arg OPT4048_400_MS  (0xAU): 400.0 ms (Total cycle: 1.6 s)
  *                  @arg OPT4048_800_MS  (0xBU): 800.0 ms (Total cycle: 3.2 s)
  * @retval None
  */
void OPT4048_SetConvTime(OPT4048_HandleTypeDef *opt4048, OPT4048_ConvTime_TypeDef convTime)
{
    // Read the current CONFIGURATION register
    uint16_t regValue = OPT4048_ReadRegister(opt4048, OPT4048_CONFIGURATION_REG);

    // Clear the CONVERSION_TIME bits using the mask (bits 6, 7, 8, and 9)
    regValue &= ~OPT4048_CONVERSION_TIME_Mask;

    // Set the new CONVERSION_TIME by shifting it to its position (CONVERSION_TIME_Pos = 6)
    // and protecting it with the mask
    regValue |= (convTime << OPT4048_CONVERSION_TIME_Pos) & OPT4048_CONVERSION_TIME_Mask;

    //Write the resulting value back to the register
    OPT4048_WriteRegister(opt4048, OPT4048_CONFIGURATION_REG, regValue);
}

void OPT4048_SetRange(OPT4048_HandleTypeDef *opt4048, OPT4048_FullScaleRange_TypeDef range)
{
    // Read the current CONFIGURATION register
    uint16_t regValue = OPT4048_ReadRegister(opt4048, OPT4048_CONFIGURATION_REG);

    // Clear the RANGE bits using the mask (bits 10,11, 12, and 13)
    regValue &= ~OPT4048_RANGE_Mask;

    // Set the new RANGE by shifting it to its position (RANGE_Pos = 10)
    // and protecting it with the mask
    regValue |= (range << OPT4048_RANGE_Pos) & OPT4048_RANGE_Mask;

    //Write the resulting value back to the register
    OPT4048_WriteRegister(opt4048, OPT4048_CONFIGURATION_REG, regValue);
}

void OPT4048_SetI2CType(OPT4048_HandleTypeDef *opt4048, OPT4048_I2CBurst_TypeDef i2cType)
{
    // Read the current CONFIGURATION_1 register
    uint16_t regValue = OPT4048_ReadRegister(opt4048, OPT4048_CONFIGURATION_1_REG);

    // Clear the I2C_BURST bit using the mask (bit 0)
    regValue &= ~OPT4048_I2C_BURST_Mask;

    // Set the new RANGE by shifting it to its position (I2C_BURST_Pos = 0)
    // and protecting it with the mask
    regValue |= (i2cType << OPT4048_I2C_BURST_Pos) & OPT4048_I2C_BURST_Mask;

    //Write the resulting value back to the register
    OPT4048_WriteRegister(opt4048, OPT4048_CONFIGURATION_1_REG, regValue); 
}

_Bool OPT4048_IsConversionReady(OPT4048_HandleTypeDef *opt4048)
{
    // Read the MASK_ENABLE Register (Address: 0x0C)
    uint16_t regValue = OPT4048_ReadRegister(opt4048, OPT4048_MASK_ENABLE_REG);
    
    // If the bit is set, (regValue & CONVERSION_READY_FLAG) will be 0x004 (which is != 0)
    if ((regValue & OPT4048_CONVERSION_READY_FLAG) != 0U)
    {
        return 1; 
    }
    return 0;
}

void opt4048GetAllAdcCodes(OPT4048_HandleTypeDef *opt4048)
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


