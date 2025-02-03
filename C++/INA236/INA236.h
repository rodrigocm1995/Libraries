/**
    *******************************************************************************************

  * @file           : INA236.h
  * @brief          : INA236 Library
    *******************************************************************************************

  * The INA236 is a 16-bit digital current monitor with an I2C interface that is
  * compliant with a wide range of digital bus voltage such as 1.2V, 1.8V, 3.3V,
  * and 5.0V. The device monitors the voltage across an external sense resistor
  * and reports values for current, bus voltage, and power
  * 
  * @details
  * High-side or low-side current sensing 
  * Operates from a 1.7V to 5.5V power supply
  * Reports current, voltage and power
  * Programmable full scale range: 20mV / 80mV
  * 16-bit ADC resolution
  * 0.1% gain error (maximum)
  * 5uV offset (maximum)
  * Low input bias current: 10nA (maximum)
  * Configurable averaging options
  * General call addressing allows convertion synchronization among devices
  * Alert limits for over and under current events
  * 1.2V compliant I2C bus, SMBus interface
  * Two device address options with a 4-pin selectable address
  * Operating temperature: -40°C y +125°C 
  * 
  * @example
  * INA236_CALIBRATION_REGISTER is calculated based on the next equation
  * SHUNT_CAL = 0.00512/(Current_LSB x Rshunt).......................(1)
  * 
  * Where:
  *   0.00512 is an internal fixed value used to ensure scaling is maintained properly
  *   Current_LSB is a selected value for the current step size in A. Must be greater than 
  *   or equal to CURRENT_LSB (minimum), but less than 8 x CURRENT_LSB (minimun) to 
  *   reduce resolution loss.
  *   The SHUNT_CAL must be divided by 4 for ADCRANGE = 1
  * 
  * CURRENT_LSB (minimum) = Maximum Expected Current / 2^15..........(2)
  * 
  * After programming the SHUNT_CAL register with the calculated value, the measured current
  * in amperes can be read from the CURRENT register. Use equation (3) to calculate the final
  * value scaled by the CURRENT_LSB:
  * 
  * Current[A] = CURRENT_LSB x CURRENT...............................(3)
  * 
  * Where:
  *   CURRENTis the value read from the CURRENT register
  * 
  * Design Parameters
  * * Power Supply Voltage (Vs)        = 3.3V
  * * Bus Supply  rail (VCM)           = 12V
  * * Average Current                  = 6A
  * * OverCurrent fault threshold      = 9A
  * * Maximum current monitored (Imax) = 10A
  * * ADC Range Selection (Vsense_max) = +-81.92mV 
  * 
  * 1. Select the Shunt Resistor
  * 
  * Rshunt < V_SENSE_MAX / I_MAX ....................................(4)
  * 
  *******************************************************************************************
  */

#ifndef INC_INA236_H_
#define INC_INA236_H_

#include <Arduino.h>
#include <Math.h>
#include <Wire.h>

#define INA236_DEFAULT_ADDRESS          0x40
#define INA236_TRIALS                   5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// Registers
#define INA236_CONFIGURATION_REGISTER   0x00
#define INA236_SHUNT_VOLTAGE_REGISTER   0x01
#define INA236_BUS_VOLTAGE_REGISTER     0x02
#define INA236_POWER_REGISTER           0x03
#define INA236_CURRENT_REGISTER         0x04
#define INA236_CALIBRATION_REGISTER     0x05
#define INA236_MASK_ENABLE_REGISTER     0x06
#define INA236_ALERT_LIMIT_REGISTER     0x07  // Contains the value used to compare to the register selected in the Mask/Enable Register to determine if a limit has been exceeded 
#define MANUFACTURER_ID_REGISTER        0x3E  // Reset Value: 5449h
#define DEVICE_ID_REGISTER              0x3F  // Reset Value: A080h

#define INA236_CONFIG_RESERVED          0x4000

// Mask_Enable Register
#define INA236_SHUNT_OVER_LIMIT         0x8000 // Setting this bit high configures the ALERT pin to be asserted if the shunt voltage conversion result exceeds the value programmed in the LIMIT register
#define INA236_SHUNT_UNDER_LIMIT        0x4000 // Setting this bit high configures the ALERT pin to be asserted if the shunt voltage conversion result is below the value programmed in the LIMIT register 
#define INA236_BUS_OVER_LIMIT           0x2000 // Setting this bit high configures the ALERT pin to be asserted if the bus voltage conversion result exceeds the value programmed in the LIMIT Register.
#define INA236_BUS_UNDER_LIMIT          0x1000 // Setting this bit high configures the ALERT pin to be asserted if the bus voltage conversion result is below the value programmed in the LIMIT register.
#define INA236_POWER_OVER_LIMIT         0x0800 // Setting this bit high configures the ALERT pin to be asserted if the power result exceeds the value programmed in ther LIMIT register.
#define INA236_CONVERSION_READY         0x0400 // Setting this bit high configures the ALERT pin to be asserted  when the Conversion Ready flag , bit 3, is asserted indicating that the device is ready for the next conversion.
#define INA236_MEM_ERROR                0x0020 // CRC or ECC error
#define INA236_ALERT_FUNCTION_FLAG      0x0010  
#define INA236_CONVERSION_READY_FLAG    0x0008
#define INA236_MATH_OVER_FLOW           0x0004 // This bit is set to '1' if an arithmetic operation resulted in an overflow error. It indicates that current and power data may be invalid
#define INA236_ALERT_POLARITY           0x0002 // Alert polarity bit sets the alert pin polarity. 0b = Normal (Active-low open drain). 1b = Inverted (active-high)
#define INA236_ALERT_LATCH_ENABLE       0X0001 // When the Alert latch Enable bit is set to Latch mode, the Alert pin and AFF bit remains active following a fault until this register flag has been read. This bit must be set to use the I2C Alert Response function   

/**
  * Enables the selection of the shunt full scale input across IN+ and IN-
  */
typedef enum {
  INA236_ADCRANGE_81DOT92_MILIVOLT    = 0x0000, // ADCRANGE = 0 (+-81.92mV). 2.5uV/LSB
  INA236_ADCRANGE_20DOT48_MILIVOLT    = 0x1000  // ADCRANGE = 1 (+-20.48mV). 625nV/LSB      
} Ina236AdcRange_t;

typedef enum { 
  INA236_1_SAMPLE                     = 0x0000,
  INA236_4_SAMPLES                    = 0x0200,
  INA236_16_SAMPLES                   = 0x0400,
  INA236_64_SAMPLES                   = 0x0600,
  INA236_128_SAMPLES                  = 0x0800,
  INA236_256_SAMPLES                  = 0x0A00, 
  INA236_512_SAMPLES                  = 0x0C00,
  INA236_1024_SAMPLES                 = 0x0E00
} Ina236Avg_t;

typedef enum {
  INA236_VBUSCT_140_US                = 0x0000,
  INA236_VBUSCT_204_US                = 0x0040,
  INA236_VBUSCT_332_US                = 0x0080,
  INA236_VBUSCT_588_US                = 0x00C0,
  INA236_VBUSCT_1100_US               = 0x0100,
  INA236_VBUSCT_2116_US               = 0x0140,
  INA236_VBUSCT_4156_US               = 0x0180,
  INA236_VBUSCT_8244_US               = 0x01C0
} Ina236VbusConvertionTime_t;

typedef enum {
  INA236_VSHCT_140_US                = 0x0000,
  INA236_VSHCT_204_US                = 0x0008,
  INA236_VSHCT_332_US                = 0x0010,
  INA236_VSHCT_588_US                = 0x0018,
  INA236_VSHCT_1100_US               = 0x0020,
  INA236_VSHCT_2116_US               = 0x0028,
  INA236_VSHCT_4156_US               = 0x0030,
  INA236_VSHCT_8244_US               = 0x0038
} Ina236ShuntConvertionTime_t;

typedef enum {
  INA236_SHUTDOWN_MODE               = 0x0000,
  INA236_SHVOLT_TRIGGERED_ONE_SHOT   = 0x0001,
  INA236_BUSVOLT_TRIGGERED_ONE_SHOT  = 0x0002,
  INA236_SHUNTBUS_TRIGGERED_ONE_SHOT = 0X0003,
  INA236_SHUNTDOWN                   = 0x0004,
  INA236_CONTINUOUS_SHUNT_VOLTAGE    = 0x0005,
  INA236_CONTINUOUS_BUS_VOLTAGE      = 0x0006,
  INA236_CONTINUOUS_SHUNTBUS_VOLTAGE = 0x0007
} Ina236Mode_t;

typedef enum {
  SOL = 1, // Shunt Over Limit
  SUL = 2, // Shunt Under Limit
  BOL = 3, // Bus Over limit
  BUL = 4, // Bus Under Limit
  POL = 5,  // Power Over Limit
  CNVR = 6 // Conversion Ready
} Ina236AlertType_t;

class INA236{
  public:
    INA236(); // Constructor. It must be defined with the same name as the class
    uint8_t  defaultInit(uint8_t devAddress, TwoWire *wire = &Wire);
    uint8_t  init(uint8_t devAddress, Ina236AdcRange_t adcRange, Ina236Avg_t samples, Ina236VbusConvertionTime_t busConvertionTime, Ina236ShuntConvertionTime_t shuntConvertionTime, Ina236Mode_t mode, TwoWire *wire = &Wire);
    uint16_t setCalibration(float rShuntValue, float maxCurrent);
    uint8_t  setMaskRegister(Ina236AlertType_t alertType);
    void     resetMaskRegister();
    void     setCurrentAlertLimit(float currentLimit);
    uint16_t writeRegister(uint8_t registerAddress, uint16_t value);
    uint16_t readRegister(uint8_t registerAddress);
    bool     dataReady();
    bool     alertFunctionFlag();
    float    getShuntVoltage();
    float    getBusVoltage();
    float    getCurrent();  
    float    getPower();

  private:
    uint8_t           _deviceAddress = INA236_DEFAULT_ADDRESS;
    float             _rShunt;
    float             _maximumCurrent;
    float             _currentLsbMin;
    uint8_t           _rangeAdc;
    float             _shuntAdcRange;
    Ina236AlertType_t _alertType
    TwoWire *         _wire; //The generic connection to user's chosen I2C hardware
};

#endif