/**
    *******************************************************************************************
  * @file           : ZTPD2210.h
  * @brief          : ZTPD2210 Library
    *******************************************************************************************

  * The ZTPD-2210 is a infrared Thermopile detector with digital calibrated output for non-contact
  * temperature detection. This sensor measurement detection range is -20°C to ~100°C
  *   
  * @details
  * Factory Calibrated temperature range
  * - Standard type detection range: -20°C ~ 100°C
  * - Normal snesor operation range: -20°C ~ 85°C
  * - Sensor storage temperature range: -40°C ~ 100°C
  * Ambient temperature compensation
  * I2C interface
  * Sleep mode for energy saving
  * 
  * @Applications
  * Non-contact thermometer
  * Temperature sensing for automotive cabin building HVAC system
  * Home appliance(Microwave oven, Refrigerator, Coffer machine, etc.)
  * Healthcare (Human body temperature detection)
  *******************************************************************************************
  */

#ifndef INC_ZTPD2210_H_
#define INC_ZTPD2210_H_


#define ZTPD2210_ADDRESS                         0x38
#define ZTPD2210_TRIALS                          5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// REGISTERS
#define ZTPD2210_TEMPERATURE_REGISTER            0xAF

typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint8_t            devAddress;
  uint64_t           fullDataTemperature;
}ZTPD2210_t;


void ZTPD2210writeRegister(ZTPD2210_t *ztpd2210, uint8_t registerAddress, uint16_t value);

uint64_t ZTPD2210readRegister(ZTPD2210_t *ztpd2210, uint8_t registerAddress);

void ZTPD2210Init(ZTPD2210_t *ztpd2210, I2C_HandleTypeDef *i2c, uint8_t devAddress);

double ZTPD2210getObjectTemperature(ZTPD2210_t *ztpd2210);

double ZTPD2210getAmbientTemperature(ZTPD2210_t *ztpd2210);

#endif
