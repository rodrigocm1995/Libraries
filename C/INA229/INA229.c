#include <INA229.h>
#include <math.h>

#define maxRegAddress 0x3F
#define MSB(u16) (((u16) & 0xFF00U) >> 8)
#define LSB(u16) ((u16) & 0xFFU)

const uint8_t INA229RegSize[maxRegAddress+1] = {
  2,2,2,2,3,3,2,3,\
  3,5,5,2,2,2,2,2,\
  2,2,0,0,0,0,0,0,\
  0,0,0,0,0,0,0,0,\
  0,0,0,0,0,0,0,0,\
  0,0,0,0,0,0,0,0,\
  0,0,0,0,0,0,0,0,\
  0,0,0,0,0,0,2,2
};

void INA229_Init(INA229_t *ina229,
                    SPI_HandleTypeDef *spiHandle, 
					GPIO_TypeDef *csPort,
                    uint16_t csPin)
{
	ina229->spiHandle = spiHandle;
	ina229->csPort    = csPort;
	ina229->csPin     = csPin;

	uint16_t config = INA229_0_S | INA229_TEMP_COMP_DIS | INA229_ADC_163DOT84_MILIVOLT;
	uint16_t adcConfig = INA229_BUS_SHUNT_TEMP_CONT | INA229_VBUS_4120_US | INA229_SHUNT_4120_US | INA229_TEMP_4120_US | INA229_1_SAMPLE;

	if (!CHECK_BIT(config,4))
	{
		ina229->adcRange = 0;
		ina229->vshuntConvFactor = INA229_VSHUNT_CONV_FACTOR_ADC_0;
	}else{
		ina229->adcRange = 1;
		ina229->vshuntConvFactor = INA229_VSHUNT_CONV_FACTOR_ADC_1;
	}

	INA229_WriteRegister(ina229, CONFIG, config);
	INA229_WriteRegister(ina229, ADC_CONFIG, adcConfig);
}

void INA229_Custom_Init(INA229_t *ina229,
		SPI_HandleTypeDef *spiHandle,
		GPIO_TypeDef *csPort,
		uint16_t csPin,
		Ina229_Conv_Dly dly,
		Ina229_Temp_Comp tpComp,
		Ina229_Adc_Range adcRange)
{
	ina229->spiHandle = spiHandle;
	ina229->csPort    = csPort;
	ina229->csPin     = csPin;

	uint16_t config = dly | tpComp | adcRange;

	if (!CHECK_BIT(config,4))
	{
		ina229->adcRange = 0;
	}else{
		ina229->adcRange = 1;
	}

	INA229_WriteRegister(ina229, CONFIG, config);
}

void INA229_ADC_Config(INA229_t *ina229,
		Ina229_Mode mode,
		Ina229_VBus_CT cbusCt,
		Ina229_Shunt_CT shuntCt,
		Ina229_Temp_CT tempCt,
		Ina229_Avg avg)
{
	uint16_t adcConfig = mode | cbusCt | shuntCt | tempCt | avg;
	INA229_WriteRegister(ina229, ADC_CONFIG, adcConfig);
}

uint8_t INA229_WriteRegister(INA229_t *ina229, uint8_t registerAddress, uint16_t value)
{
  uint8_t txBuf[3] = {0}; // All Writable registers are 2 bytes
  uint8_t rxBuf[3] = {0};

  txBuf[0] = registerAddress << 2; // Address + write bit (ending in 0)
  txBuf[1] = MSB(value);
  txBuf[2] = LSB(value);

  HAL_GPIO_WritePin(ina229->csPort, ina229->csPin, GPIO_PIN_RESET);
  uint8_t status = (HAL_SPI_TransmitReceive(ina229->spiHandle, txBuf, rxBuf, 3, HAL_MAX_DELAY) == HAL_OK);
  //while((HAL_SPI_TransmitReceive(ina229->spiHandle, txBuf, rxBuf, 3, HAL_MAX_DELAY) != HAL_OK));
  HAL_GPIO_WritePin(ina229->csPort, ina229->csPin, GPIO_PIN_RESET);

  return status;
}

uint64_t INA229_ReadRegister(INA229_t *ina229, uint8_t registerAddress)
{
  uint64_t value;
  int i;

  uint8_t txBuf[1] = {0};
  uint8_t rxBuf[6] = {0}; //max buf size

  txBuf[0] = (registerAddress << 2) | 0x01; //Address + read bit (ending in 1)

  HAL_GPIO_WritePin(ina229->csPort, ina229->csPin, GPIO_PIN_RESET);
  //uint8_t status = (HAL_SPI_TransmitReceive(ina229->spiHandle, txBuf, rxBuf, INA229RegSize[registerAddress]+1, 100) == HAL_OK);
  while((HAL_SPI_TransmitReceive(ina229->spiHandle, txBuf, rxBuf, INA229RegSize[registerAddress]+1, HAL_MAX_DELAY) != HAL_OK));
  while(HAL_SPI_GetState(ina229->spiHandle) != HAL_SPI_STATE_READY);
  HAL_GPIO_WritePin(ina229->csPort, ina229->csPin, GPIO_PIN_RESET);

  //Combine bytes
  value = 0;

  for (i = 1; i < INA229RegSize[registerAddress]+1; i++)
  {
    value = (value << 8) | rxBuf[i];
  }

  return value;
}

void INA229_Reset_Registers(INA229_t *ina229)
{
	uint16_t config = 0;
	_SET_BIT(config, 15);
	_SET_BIT(config, 14);
	INA229_WriteRegister(ina229, CONFIG, config);
}

void INA229_SetCalibration(INA229_t *ina229,
		double shuntResistor,
		uint16_t maxCurrent)
{
	ina229->shuntResistor = shuntResistor;
	ina229->maximumCurrent = maxCurrent;
	ina229->currentLsb = maxCurrent / pow(2,19);
	ina229->shuntCal = INA229_SCALING_FACTOR * (ina229->currentLsb) * shuntResistor;

	if (ina229->adcRange)
	{
		ina229->shuntCal = (ina229->shuntCal) * 4;
	}

	INA229_WriteRegister(ina229, SHUNT_CAL, (uint16_t)(ina229->shuntCal));
}

double INA229_Get_Current(INA229_t *ina229)
{
	double current = INA229_ReadRegister(ina229, CURRENT);
	current = current * ina229->currentLsb;

	if (current <= 1000.0)
	{
		current = current * 1000;
	}

	return current;
}

double INA229_Get_Shunt_Voltage(INA229_t *ina229)
{
	double shuntVoltage = INA229_ReadRegister(ina229, VSHUNT);

}

double INA229_Get_Power(INA229_t *ina229)
{
	double power = INA229_ReadRegister(ina229, POWER);
	power = 3.2 * power * ina229->currentLsb;

	if (power <= 1000.0)
	{
		power *= 1000;
	}

	return power;
}

double INA229_Get_Energy(INA229_t *ina229)
{
	double energy = INA229_ReadRegister(ina229, ENERGY);
	energy = 16.0 * 3.2 * ina229->currentLsb * energy;

	return energy;
}

double INA229_Get_Charge(INA229_t *ina229)
{
	double charge = INA229_ReadRegister(ina229, CHARGE);
	charge = ina229->currentLsb * charge;

	return charge;
}

double INA229_Get_Temperature(INA229_t *ina229)
{
	double temperature = INA229_ReadRegister(ina229, DIETEMP);
	temperature = INA229_TEMP_CONV_FACTOR * temperature;

	return temperature;
}


_Bool INA229_Get_Energy_OverFlow(INA229_t *ina229)
{
	uint16_t alertValue = INA229_ReadRegister(ina229, DIAG_ALERT);
	_Bool isEnergyOverflow = CHECK_BIT(alertValue, 11);

	return isEnergyOverflow;
}

_Bool INA229_Get_Charge_OverFlow(INA229_t *ina229)
{
	uint64_t alertValue = INA229_ReadRegister(ina229, DIAG_ALERT);
	_Bool isChargeOverflow = CHECK_BIT(alertValue, 10);

	return isChargeOverflow;
}

_Bool INA229_Data_Ready(INA229_t *ina229)
{
	uint16_t conversionReady = INA229_ReadRegister(ina229, DIAG_ALERT);
	_Bool isDataReady = CHECK_BIT(conversionReady, 1);

	return isDataReady;
}

