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

#include <Arduino.h>
#include <Wire.h>

#define ZTPD2210_ADDRESS                         0x38
#define ZTPD2210_TRIALS                          5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// REGISTERS
#define ZTPD2210_TEMPERATURE_REGISTER            0xAF

// Helps to reduce average supply current when continuous
// conversions are not necessary

class ZTPD2210{
  public:
    ZTPD2210(); // Constructor. It must be defined with the same name as the class
    void     init(uint8_t devAddress, TwoWire *wire = &Wire);
    uint8_t  writeRegister(uint8_t registerAddress, uint8_t value);
    uint64_t readRegister(uint8_t registerAddress);
    float    getObjectTemperature();
    float    getAmbientTemperature();

  private:
    uint8_t _deviceAddress = ZTPD2210_ADDRESS;
    uint64_t _fullDataTemperature;
    TwoWire * _wire; 
};

#endif