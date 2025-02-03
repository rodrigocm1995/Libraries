/**
  ******************************************************************************
  * @file           : INA219.h
  * @brief          : INA219 Library
  ******************************************************************************
  * The INA219 is a current shunt and power monitor with a I2C or SMBUS-compati-
  * ble interface. The device monitors both both shunt voltage drop and bus sup-
  * ply voltagewith programmable conversion times and filtering. A programmable 
  * calibration value, combined with an internal multiplier, enables direct read
  * outs of current in amperes. An additional multiplying register calculates po
  * wer in watts.
  * 
  * @details
  * Senses Bus Voltages from 0 to 26 V
  * Reports Current, Voltage, and Power
  * 16 Programmable Addresses
  * High Accuracy: 0.5% (Maximum) Over Temperature (INA219B)
  * Filtering Options
  * Calibration Registers
  * SOT23-8 and SOIC-8 Packages
  * 
  * @example
  * INA219_CALIBRATION_REGISTER is calculated based on the next equation
  * Cal = trunc(0.04096/(Current_LSB * R_Shunt)).....................(1)
  * 
  * Where:
  *   0.04096 is an internal fixed value used to insure scaling
  *   Current_LSB = Maximum Expected Current / 2^15..................(2)
  * 
  * If user assumes a maximum current of 1A and a R_Shunt of 100mOhm then using
  * (2)
  * Current_LSB = 1/2^15 = 30.51 uA/bit .............................(3)
  * 
  * Replacing (3) in (1) yields
  * Cal = (0.04096)/(30.51exp(-6) * 100exp(-3)) = 13425
  * This value must be written in the 16-bit Calibration Register
  * 
  * 
  ******************************************************************************
  */

#ifndef _LIB_MYINA219_
#define _LIB_MYINA219_

#include <Arduino.h>
#include <Math.h>
#include <Wire.h>

// Addresses
#define INA219_I2C_ADDRESS1                      0x40
#define INA219_I2C_ADDRESS2                      0x41
#define INA219_I2C_ADDRESS3                      0x44
#define INA219_I2C_ADDRESS4                      0x45
// end of addresses

#define INA219_TRIALS                             5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// Registers
#define INA219_CONFIGURATION_REG                  0x00   // All-register reset, settings for bus voltage range, PGA gain, ADC Resolution/averaging
#define INA219_SHUNTVOLTAGE_REG                   0x01   // Shunt voltage measurement data
#define INA219_BUSVOLTAGE_REG                     0x02   // Bus voltage measurement data
#define INA219_POWER_REG                          0x03   // power measurement data
#define INA219_CURRENT_REG                        0x04   // Contains the value of the current flowing through the shunt resistor
#define INA219_CALIBRATION_REG                    0x05   // Sets full-scale range and LSB of current and power measurements. Overall system calibration
// End of registers

#define INA219_BUS_VOLTAGE_RANGE_16V            0x0000  
#define INA219_BUS_VOLTAGE_RANGE_32V            0x2000
#define INA219_PGA_GAIN_ONE                     0x0000
#define INA219_PGA_GAIN_ONE_HALF                0x0800
#define INA219_PGA_GAIN_ONE_QUARTER             0x1000
#define INA219_PGA_GAIN_ONE_EIGHTH              0x1800
#define INA219_BUS_ADC_9BIT_RESOLUTION          0x0000
#define INA219_BUS_ADC_10BIT_RESOLUTION         0x0080
#define INA219_BUS_ADC_11BIT_RESOLUTION         0x0100
#define INA219_BUS_ADC_12BIT_RESOLUTION         0x0180
#define INA219_BUS_ADC_2_SAMPLES                0x0480      
#define INA219_BUS_ADC_4_SAMPLES                0x0500
#define INA219_BUS_ADC_8_SAMPLES                0x0580
#define INA219_BUS_ADC_16_SAMPLES               0x0600
#define INA219_BUS_ADC_32_SAMPLES               0x0680
#define INA219_BUS_ADC_64_SAMPLES               0x0700
#define INA219_BUS_ADC_128_SAMPLES              0x0780

#define INA219_SHUNT_VOLTAGE_9BIT_RESOLUTION    0x0000
#define INA219_SHUNT_VOLTAGE_10BIT_RESOLUTION   0x0008
#define INA219_SHUNT_VOLTAGE_11BIT_RESOLUTION   0x0010
#define INA219_SHUNT_VOLTAGE_12BIT_RESOLUTION   0x0018
#define INA219_SHUNT_VOLTAGE_12BIT_RESOLUTION   0x0018
#define INA219_SHUNT_VOLTAGE_2_SAMPLES          0x0048
#define INA219_SHUNT_VOLTAGE_4_SAMPLES          0x0050
#define INA219_SHUNT_VOLTAGE_8_SAMPLES          0x0058
#define INA219_SHUNT_VOLTAGE_16_SAMPLES         0x0060
#define INA219_SHUNT_VOLTAGE_32_SAMPLES         0x0068
#define INA219_SHUNT_VOLTAGE_64_SAMPLES         0x0070
#define INA219_SHUNT_VOLTAGE_128_SAMPLES        0x0078

typedef enum {
  INA219_BUSVOLTAGERANGE_16V                  = 0x0000, 
  INA219_BUSVOLTAGERANGE_32V                  = 0x2000
} BusVoltageRange_t;

typedef enum {
  INA219_PGAGAIN_40_MILI_VOLT                 = 0x0000, // Range +- 40mV
  INA219_PGAGAIN_80_MILI_VOLT                 = 0x0800, // Range +- 80mV
  INA219_PGAGAIN_160_MILI_VOLT                = 0x1000, // Range +- 160mV
  INA219_PGAGAIN_320_MILI_VOLT                = 0x1800  // Range +- 320mV 
} ShuntPGAGain_t;

typedef enum {
  INA219_BUS_ADC_9_BIT_RESOLUTION             = 0x0000, // 84 us
  INA219_BUS_ADC_10_BIT_RESOLUTION            = 0x0080, // 148 us
  INA219_BUS_ADC_11_BIT_RESOLUTION            = 0x0100, // 276 us
  INA219_BUS_ADC_12_BIT_RESOLUTION            = 0x0180, // 532 us
  INA219_BUS_ADC_2SAMPLES                     = 0x0480, // 1.06 us
  INA219_BUS_ADC_4SAMPLES                     = 0x0500, // 2.13 ms
  INA219_BUS_ADC_8SAMPLES                     = 0x0580, // 4.26 ms
  INA219_BUS_ADC_16SAMPLES                    = 0x0600, // 8.51 ms
  INA219_BUS_ADC_32SAMPLES                    = 0x0680, // 17.02 ms
  INA219_BUS_ADC_64SAMPLES                    = 0x0700, // 34.05 ms
  INA219_BUS_ADC_128SAMPLES                   = 0x0780  // 68.10 ms
} BusADCResolution_t;

typedef enum {
  INA219_SHUNT_ADC_9_BIT_RESOLUTION           = 0x0000, // 84 us
  INA219_SHUNT_ADC_10_BIT_RESOLUTION          = 0x0008, // 148 us
  INA219_SHUNT_ADC_11_BIT_RESOLUTION          = 0x0010, // 276 us
  INA219_SHUNT_ADC_12_BIT_RESOLUTION          = 0x0018, // 532 us
  INA219_SHUNT_ADC_2SAMPLES                   = 0x0048, // 1.06 ms
  INA219_SHUNT_ADC_4SAMPLES                   = 0x0050, // 2.13 ms
  INA219_SHUNT_ADC_8SAMPLES                   = 0x0058, // 4.26 ms
  INA219_SHUNT_ADC_16SAMPLES                  = 0x0060, // 8.51 ms
  INA219_SHUNT_ADC_32SAMPLES                  = 0x0068, // 17.02 ms
  INA219_SHUNT_ADC_64SAMPLES                  = 0x0070, // 34.05 ms
  INA219_SHUNT_ADC_128SAMPLES                 = 0x0078  // 68.10 ms
} ShuntADCResolution_t;

typedef enum {
  INA219_POWERDOWN_MODE                       = 0x0000,
  INA219_SHUNTVOLTAGETRIG_MODE                = 0x0001,
  INA219_BUSVOLTAGETRIG_MODE                  = 0x0002,
  INA219_SHUNTBUS_TRIG_MODE                   = 0x0003,
  INA219_ADCOFF_DISABLED_MODE                 = 0x0004,
  INA219_SHUNTVOLTAGE_CONTINUOUS_MODE         = 0x0005,
  INA219_BUSVOLTAGE_CONTINUOUS_MODE           = 0x0006,
  INA219_SHUNTBUS_CONTINUOUS_MODE             = 0x0007
} Mode_t;

class INA219{
  public:
    INA219(); // Constructor. It must be defined with the same name as the class
    bool     begin();
    bool     isConnected();
    uint16_t defaultInit(uint8_t devAddress, TwoWire *wire = &Wire);
    uint16_t init(uint8_t devAddress, BusVoltageRange_t busvoltageRange, ShuntPGAGain_t pgaGain, BusADCResolution_t busAdcResolution, ShuntADCResolution_t shuntAdcResolution, Mode_t mode, TwoWire *wire = &Wire);
    uint16_t getConfigRegister();
    uint16_t setCalibration(float rShuntValue, float maxCurrent);
    uint16_t getCalibrationRegister();
    uint8_t  correctedFullScaleCalibration(uint16_t calibrationValue, float inaCurrent, float measuredShuntCurrent);
    uint16_t writeRegister(uint8_t registerAddress, uint16_t value);
    uint16_t readRegister(uint8_t registerAddress);
    bool     dataReady();
    float    readShuntVoltage();
    float    readBusVoltage();
    float    readCurrent();  
    float    readPower();

  private:
    uint8_t _deviceAddress = INA219_I2C_ADDRESS2;
    float   _rShunt;
    float   _maximumCurrent;
    float   _currentLSB;
    TwoWire * _wire; //The generic connection to user's chosen I2C hardware
};

#endif