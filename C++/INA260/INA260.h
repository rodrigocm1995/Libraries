/**
  ******************************************************************************
  * @file           : INA260.h
  * @brief          : INA260 Library
  ******************************************************************************
  * The INA260 is a digital-output, current, power, and voltage monitor with an I2C and
  * SMBus compatible interface with an integrated precision shunt resistor. it enables
  * high-accuracy current and power measurements and over-current detection at common-
  * mode voltages that can vary from 0V to 36V, independent of the supply voltage. The
  * device is a bidirectional, low- or high-side, current-shunt monitor that measures
  * current flowing through the internal current-sensing resistor. The integration of
  * the precision current-sensing resistor provides calibration-equivalent measurement 
  * accuracy with ultra-low temperature drift performance and ensures that an optimized 
  * Kelvin layout for the sensing resistor is always obtained.
  * 
  * The INA260 features up to 16 programmable addresses on the I2C compatible interface.
  * The digital interface allows programmable alert thresholds, analog-to-digital converter
  * (ADC) conversion times, and averaging. To facilitate ease of use, an internal 
  * multiplier enables direct readouts of current in amperes and power in watts.
  * 
  * @details
  * Precision Integrated Shunt Resistor:
  * - Current Snese Resistance: 2mΩ
  * - Tolerance Equivalent to 0.1%
  * - 15-A Continuous From -40°C to +85°C
  * - 10 ppm/°C Temperature Coefficient (0°C to +125°C)
  * Senses  Bus Voltages from 0 V to 36 V
  * High-Side or Low-Side Sensing
  * Reports Current, Voltage, and Power
  * High Accuracy:
  * - 0.15% System Gain Error (Maximum)
  * - 5-mA Offset (Maximum)
  * Configurable Averaging Options
  * 16 Programmable Addresses
  * Operates From a 2.7V to 5.5V Power Supply
  * 16 pin, TSSOP Package
  * 
  * @example
  *
  * 
  ******************************************************************************
  */

#ifndef INC_INA260_H_
#define INC_INA260_H_

#include <Arduino.h>
#include <Math.h>
#include <Wire.h>

// Address
#define INA260_I2C_ADDRESS                        0x40 // A1 = GND, A0 = GND

#define INA260_TRIALS                             5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// Registers
#define INA260_CONFIGURATION_REG                  0x00   // All-register reset, shunt voltage and bus voltage ADC conversion times and averaging, operating mode.
#define INA260_CURRENT_REG                        0x01   // Contains the value of the current flowing through the shunt resistor
#define INA260_BUS_VOLTAGE_REG                    0x02   // Bus voltage measurement data
#define INA260_POWER_REG                          0x03   // Contains the value of the calculated power being delivered to the load
#define INA260_MASK_ENABLE_REG                    0x06   // Alert configuration and Conversion Ready flag
#define INA260_ALERT_LIMIT_REG                    0x07   // Contains the limit value ttto compare to the selected Alert function
#define INA260_MANUFACTURER_ID_REG                0xFE   // Contains unique manufacturer identification number - 5549h
#define INA260_DIE_ID_REG                         0xFF   // Contains unique die identification number - 2270h

/** Averaging Mode
  * @brief Determines the number of samples that are collected and averaged.
  *        The following shows all the AVG bit settings and related number 
  *        of averages for each bit setting. 
  */
typedef enum {
  INA260_1_SAMPLE                             = 0x0000,
  INA260_4_SAMPLES                            = 0x0200,
  INA260_16_SAMPLES                           = 0x0400,
  INA260_64_SAMPLES                           = 0x0600,
  INA260_128_SAMPLES                          = 0x0800,
  INA260_256_SAMPLES                          = 0x0A00,
  INA260_512_SAMPLES                          = 0x0C00,
  INA260_1024_SAMPLES                         = 0x0E00,
} Ina260Avg;

/** Bus Voltage Conversion Time
  * @brief Sets the conversion time for the bus voltage measurement. The
  *        following shows the VBUSCT bit options and related conversion
  *        times for each bit setting. 
  */
typedef enum {
  INA260_VBUS_140US                           = 0x0000,
  INA260_VBUS_204US                           = 0x0040,
  INA260_VBUS_332US                           = 0x0080,
  INA260_VBUS_588US                           = 0x00C0,
  INA260_VBUS_1100US                          = 0x0100,
  INA260_VBUS_2116US                          = 0x0140,
  INA260_VBUS_4156US                          = 0x0180,
  INA260_VBUS_8244US                          = 0x01C0,
} Ina260BusIshuntCt;

/** Shunt Current Conversion Time
  * @brief The following shows the ISHCT bit options and related conversion 
  *        times for each bit setting. 
  */
 typedef enum {
  INA260_SHUNT_140US                           = 0x0000,
  INA260_SHUNT_204US                           = 0x0008,
  INA260_SHUNT_332US                           = 0x0010,
  INA260_SHUNT_588US                           = 0x0018,
  INA260_SHUNT_1100US                          = 0x0020,
  INA260_SHUNT_2116US                          = 0x0028,
  INA260_SHUNT_4156US                          = 0x0030,
  INA260_SHUNT_8244US                          = 0x0038,
} Ina260BusVoltageCt;

/** Operating Mode
  * @brief Selects continuous, triggered, or power-down mode of operation. These
  *        bits default to continuous shnt and bus measurement mode. the following
  *        shows mode settings. 
  */
typedef enum {
  INA260_POWERDOWN                             = 0x0000,
  INA260_SHUNT_CURRENT_TRIG                    = 0x0001,
  INA260_VBUS_TRIG                             = 0x0002,
  INA260_ISHUNT_VBUS_TRIG                      = 0x0003,
  INA260_ISHUNT_CONTINUOUS                     = 0x0005,
  INA260_VBUS_CONTINUOUS                       = 0x0006,
  INA260_ISHUNT_VBUS_CONTINUOUS                = 0x0007,
} Ina260Mode;

class INA260{
  public:
    INA260(); // Constructor. It must be defined with the same name as the class
    bool     begin();
    bool     isConnected();
    void     reset();

    void     init(uint8_t devAddress, TwoWire *wire = &Wire);
    void     init(uint8_t devAddress, Ina260Avg samples, Ina260BusVoltageCt vBusCt, Ina260BusIshuntCt iShuntCt, Ina260Mode mode, TwoWire *wire = &Wire);

    uint16_t getConfig();

    float getCurrent();
    float getBusVoltage()
    float getPower()

    int16_t checkPolarity(int16_t value);
    bool    isReady();



  private:
    uint8_t _deviceAddress = INA260_I2C_ADDRESS2;
    float   _rShunt;
    float   _maximumCurrent;
    float   _currentLSB;
    TwoWire * _wire; //The generic connection to user's chosen I2C hardware
};

#endif
