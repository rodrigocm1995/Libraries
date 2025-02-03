/**
    *******************************************************************************************
  * @file           : BMP180.h
  * @brief          : BMP180 Library
    *******************************************************************************************

  * The BMP180 consist of a piezo-resitive sensor, an analog to digital converter and a control
  * unit with EEPROM and serial I2C interface. The BMP180 delivers the uncompensated value of 
  * pressure and temperature. The EEPROM has stored 176 bit of individual calibration data. This
  * is used to compensate offset, temperature dependence and other parameters of the sensor. 
  * 
  * @details
  * UP = pressure data (16 to 19 bits)
  * UT = temperature data (16 bits)
  * Pressure range: 300 ... 1100hPa (+9000m ... -500m relating to sea level)
  * Temperature measurement included (-40 to +85°C)
  * I2C interface
  * Fully calibrated
  * Operates from a 1.8 to 3.6V power supply
  * Lower power: 5µA at 1 sample/sec. in standard mode
  * Low noise 0.06hPa (0.5m) in ultra low power mode. 0.02hPa (0.17m advanced resolution mode) 
  * 
  * @TypicalApplications
  * Enhancement of GPS navigation (dead-reckoning, slope detection, etc.)
  * In- and out-door navigation
  * Leisure and sports
  * Weather forecast
  * Vertical velocity indication (rise/sink speed)
  * 
  * @example
  * INA236_CALIBRATION_REGISTER is calculated based on the next equation
  * SHUNT_CAL = 0.00512/(Current_LSB x Rshunt).......................(1)
  * 
  *******************************************************************************************
  */


#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include <Arduino.h>
#include <Wire.h>

// MACRO TO CHECK IF A CERTAIN BIT IS SET
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// ADDRESSES
#define BMP180_ADDRESS                          0x77

// Calibration Coefficients registers (16 bits)
#define BMP180_CAL_COEFF_AC1   0xAA
#define BMP180_CAL_COEFF_AC2   0xAC
#define BMP180_CAL_COEFF_AC3   0xAE
#define BMP180_CAL_COEFF_AC4   0xB0
#define BMP180_CAL_COEFF_AC5   0xB2
#define BMP180_CAL_COEFF_AC6   0xB4
#define BMP180_CAL_COEFF_B1    0xB6
#define BMP180_CAL_COEFF_B2    0xB8  
#define BMP180_CAL_COEFF_MB    0xBA
#define BMP180_CAL_COEFF_MC    0xBC
#define BMP180_CAL_COEFF_MD    0xBE

// Global memory map
#define OUT_XLSB   0xF8
#define OUT_LSB    0xF7
#define OUT_MSB    0xF6
#define CTRL_MEAS  0xF4
#define SOFT_RESET 0xE0
#define DEVICE_ID  0xD0

#define STD_PRESSURE 1013.25 //hPa

typedef enum {
  ULTRA_LOW_POWER       = 0x00,
  STANDARD              = 0x01,
  HIGH_RESOLUTION       = 0x02,
  ULTRA_HIGH_RESOLUTION = 0x03
} AccuracyMode_t;



class BMP180{
  public:
    BMP180(); // Constructor. It must be defined with the same name as the class
    void     init(uint8_t devAddress, TwoWire *wire = &Wire);
    void     customInit(uint8_t devAddress, AccuracyMode_t accuracyMode, TwoWire *wire = &Wire);
    uint8_t  writeRegister8(uint8_t registerAddress, uint8_t value);
    uint8_t  writeRegister16(uint8_t registerAddress, uint16_t value);
    uint8_t  readRegister8(uint8_t registerAddress);
    uint16_t readRegister16(uint8_t registerAddress);
    uint32_t readRegister24(uint8_t registerAddress);
    bool     begin(uint8_t devAddress, TwoWire *wire = &Wire);
    void     setCalibrationCoefficients();
    uint8_t  getId();
    int16_t  getCalibrationCoefficients(int16_t *calCoeff);
    uint16_t getUncompensatedTemperature();
    long     getTemperature();
    long     getUncompensatedPressure();
    long     getAltitude();
  
  private:
    uint8_t        _deviceAddress = BMP180_ADDRESS;
    AccuracyMode_t _mode;
    int16_t        _bmpAC1;
    int16_t        _bmpAC2;
    int16_t        _bmpAC3;
    int16_t        _bmpAC4;
    int16_t        _bmpAC5;
    int16_t        _bmpAC6;
    int16_t        _bmpB1;
    int16_t        _bmpB2;
    int16_t        _bmpMB;
    int16_t        _bmpMC;
    int16_t        _bmpMD;
    TwoWire *      _wire; //The generic connection to user's chosen I2C hardware
};

#endif