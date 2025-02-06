/**
    *******************************************************************************************
  * @file           : INA238.h
  * @brief          : INA238 Library
    *******************************************************************************************

  * The INA238 is an ultra-precise digital power monitor with a 16-bit delta-sigma ADC specifically
  * designed for current-sensing applications. The device can measure a full-scale input of
  * ±163.84mV or ±40.96mV across a resistive shunt sense element with common-mode voltage support
  * from -0.3V to +85V.
  * 
  * The INA238 reports current, bus voltage, temperature, and power, all while performing the
  * needed calculations in the background. The integrated temperature sensor is ±1°C accurate
  * for die temperature measurement and is useful in monitoring the system ambient temperature.
  * 
  * The low offset and gain drift desing of the INA238 allows the device to be used in precise
  * systems that do not urdergo multi-temperature calibration during manufacturing. Further, the
  * very low offset voltage and noise allow for use in A and KA sensing applications and provide 
  * a wide dynamic range without significant power dissipation losses on the sensing shunt element.  
  *  
  * 
  * @details
  * High resolution, 16-bit delta-sigma ADC
  * Current monitoring accurary:
  * - Offset voltage: ±5µV (maximum)
  * - Offset drift: ±0.02µV/°C (maximum)
  * - Gain error: ±0.1% (maximum)
  * - Gain error drift: ±25 ppm/°C (maximum)
  * - Common mode rejection: 140 dB (minimum)
  * Power monitoring accuracy:
  * - 0.7% full scale, -40°C to +125°C (maximum)
  * Fast allert response: 75 µs
  * Wide common-mode range: -0.3V to +85V
  * Bus voltage sense input: 0V to 85V
  * Shunt full-scale differential range: ±163.84mV / ±40.96mV
  * Input bias current: 2.5nA (maximum)
  * Temperature sensor: ±1°C (maximum at 25°C)
  * Programmable conversion time and averaging
  * 2.94-MHz high-speed I2C interface with 16 pin selectable addresses
  * Operates from a 2.7V to 5.5V supply:
  * - Operational current: 640µA (typical)
  * - Shuntdown current: 5µA (maximum)
  * 
  * @example
  * INA238_CALIBRATION_REGISTER is calculated based on the next equation
  * SHUNT_CAL = 819.2 × 10­^6 × CURRENT_LSB × Rshunt..........................(1)
  * 
  * Where:
  * - 819.2 × 10­^6 is an internal fixed value used to ensure scaling is maintained properly.
  * - The SHUNT_CAL (ec. 1) must be multiplied by 4 for ADCRANGE = 1.
  * - Current_LSB is the LSB step size for the CURRENT register where the current in Amperes
  *   is stored. The value of the CURRENT_LSB is based on the maximum expected current as 
  *   shown in Equation 2, and it directly defines the resolution of the CURRENT register.
  * 
  * CURRENT_LSB (minimum) = Maximum Expected Current / 2^15..................(2)
  * 
  * After programming the SHUNT_CAL register with the calculated value, the measured current
  * in amperes can be read from the CURRENT register. Use equation (3) to calculate the final
  * value scaled by the CURRENT_LSB:
  * 
  * Current[A] = CURRENT_LSB x CURRENT.......................................(3)
  * 
  * Where:
  *   CURRENTis the value read from the CURRENT register
  * 
  * Design Parameters
  * * Power Supply Voltage (Vs)                     = 5V
  * * Bus Supply  rail (VCM)                        = 48V
  * * Bus supply rail over voltage fault threshold  = 52V
  * * Average Current                               = 6A
  * * OverCurrent fault threshold (Imax)            = 10A
  * * ADC Range Selection (Vsense_max)              = ±163.84mV 
  * 
  * 1. Select the Shunt Resistor
  * 
  * Rshunt < V_SENSE_MAX / I_MAX ............................................(4)
  * Rshunt < 163.84mV / 10A = 16.384 mΩ. Closest standart resistor is 16.2 mΩ
  * 
  *******************************************************************************************
  */

  #ifndef INC_INA238_H_
  #define INC_INA238_H_
  
  #include <Arduino.h>
  #include <Math.h>
  #include <Wire.h>
  
  #define INA238_DEFAULT_ADDRESS          0x40
  #define INA238_TRIALS                   5
  #define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
  
  #define MAX_REG_ADDRESS                 0x3F
  
  // REGISTERS
  #define INA238_CONFIG_REGISTER          0x00 // Configuration
  #define INA238_ADC_CONFIG_REGISTER      0x01 // ADC Configuration
  #define INA238_SHUNT_CAL_REGISTER       0x02 // Shunt Calibration
  #define INA238_VSHUNT_REGISTER          0x04 // Shunt Voltage Measurement
  #define INA238_VBUS_REGISTER            0x05 // Bus Voltage Measurement
  #define INA238_DIETEMP_REGISTER         0x06 // Temperature measurement
  #define INA238_CURRENT_REGISTER         0x07 // Current Result
  #define INA238_POWER_REGISTER           0x08 // Power Result
  #define INA238_DIAG_ALERT_REGISTER      0x0B // Diagnostic Flags and Alert
  #define INA238_SOVL_REGISTER            0x0C // Shunt Overvoltage Threshold
  #define INA238_SUVL_REGISTER            0x0D // Shunt Undervoltage Threshold
  #define INA238_BOVL_REGISTER            0x0E // Bus overvoltage Threshold
  #define INA238_BUVL_REGISTER            0x0F // Bus undervoltage Threshold
  #define INA238_TEMP_LIMIT_REGISTER      0x10 // Temperature over-limit Threshold
  #define INA238_POWER_LIMIT_REGISTER     0x11 // Power Over-limit Threshold
  #define MANUFACTURER_ID_REGISTER        0x3E // Manufacturer ID
  #define DEVICE_ID_REGISTER              0x3F // Device ID
  
  #define INA238_CONFIG_RESERVED          0x0000
  
  //CONFIG Register
  typedef enum {
    INA238_0_SEC_DELAY                    = 0x0000,   
    INA238_2_MSEC_DELAY                   = 0x0040,
    INA238_510_MSEC_DELAY                 = 0x3FC0,
  } Ina238ConvDly;
  
  typedef enum {
    INA238_163_DOT_84_MV                  = 0x0000, 
    INA238_40_DOT_96_MV                   = 0x0010,
  } Ina238AdcRange;
  
  //ADC-CONFIG register
  typedef enum {
    INA238_SHUNTDOWN                      = 0x0000,
    INA238_BUSVOLTAGE_SINGLE_SHOT         = 0x1000, 
    INA238_SVOLTAGE_SINGLE_SHOT           = 0x2000,
    INA238_SBVOLTAGE_SINGLE_SHOT          = 0x3000,
    INA238_TEMP_SINGLE_SHOT               = 0x4000,
    INA238_TEMP_BVOLTAGE_SINGLE_SHOT      = 0x5000,
    INA238_TEMP_SVOLTAGE_SINGLE_SHOT      = 0x6000,
    INA238_TEMP_SBVOLTAGE_SINGLE_SHOT     = 0x7000,
    INA238_CONT_BVOLTAGE                  = 0x9000,
    INA238_CONT_SVOLTAGE                  = 0xA000,
    INA238_CONT_SBVOLTAGE                 = 0xB000,
    INA238_CONT_TEMP                      = 0xC000,
    INA238_CONT_TEMP_BVOLTAGE             = 0xD000,
    INA238_CONT_TEMP_SVOLTAGE             = 0xE000,
    INA238_CONT_TEMP_SBVOLTAGE            = 0xF000,
  } Ina238Mode;
  
  typedef enum {
    INA238_50_USEC_VBUS                   = 0x0000,
    INA238_84_USEC_VBUS                   = 0x0200,
    INA238_150_USEC_VBUS                  = 0x0400,
    INA238_280_USEC_VBUS                  = 0x0600,
    INA238_540_USEC_VBUS                  = 0x0800,
    INA238_1052_USEC_VBUS                 = 0x0A00,
    INA238_2074_USEC_VBUS                 = 0x0C00,
    INA238_4120_USEC_VBUS                 = 0x0E00,
  } Ina238VbusConvTime;
  
  typedef enum {
    INA238_50_USEC_VSH                   = 0x0000,
    INA238_84_USEC_VSH                   = 0x0040,
    INA238_150_USEC_VSH                  = 0x0080,
    INA238_280_USEC_VSH                  = 0x00C0,
    INA238_540_USEC_VSH                  = 0x0100,
    INA238_1052_USEC_VSH                 = 0x0140,
    INA238_2074_USEC_VSH                 = 0x0180,
    INA238_4120_USEC_VSH                 = 0x01C0,
  } Ina238VshuntConvTime;
  
  typedef enum {
    INA238_50_USEC_TEMP                   = 0x0000,
    INA238_84_USEC_TEMP                   = 0x0008,
    INA238_150_USEC_TEMP                  = 0x0010,
    INA238_280_USEC_TEMP                  = 0x0018,
    INA238_540_USEC_TEMP                  = 0x0020,
    INA238_1052_USEC_TEMP                 = 0x0028,
    INA238_2074_USEC_TEMP                 = 0x0030,
    INA238_4120_USEC_TEMP                 = 0x0038,
  } Ina238TempConvTime;
  
  typedef enum {
    INA238_1_COUNT                        = 0x0000,
    INA238_4_COUNTS                       = 0x0001,
    INA238_16_COUNTS                      = 0x0002,
    INA238_64_COUNTS                      = 0x0003,
    INA238_128_COUNTS                     = 0x0004,
    INA238_256_COUNTS                     = 0x0005,
    INA238_512_COUNTS                     = 0x0006,
    INA238_1024_COUNTS                    = 0x0007,
  } Ina238Avg;
  
  
  class INA238{
    public:
      INA238(); // Constructor. It must be defined with the same name as the class
      bool     begin(uint8_t devAddress, TwoWire *wire = &Wire);
      bool     isConnected();
  
      void     init();
      void     init(uint8_t devAddress, TwoWire *wire = &Wire);
      void     init(uint8_t devAddress, Ina238ConvDly convDly, Ina238AdcRange adcRange, TwoWire *wire = &Wire);
      void     initAdcConfig();
      void     initAdcConfig(Ina238Mode mode, Ina238VbusConvTime vbusCt, Ina238VshuntConvTime vShuntCt, Ina238TempConvTime tempCt, Ina238Avg avg);
      uint16_t setCalibration(float rShuntValue, float maxCurrent);
  
      uint8_t  setShuntUnderVoltage();
      uint8_t  setShuntOverVoltage();
      int16_t  setShuntUnderVoltage(float shuntUnderVoltage);
      int16_t  setShuntOverVoltage(float shuntOverVoltage);
  
      uint8_t  setBusUnderVoltage();
      uint8_t  setBusOverVoltage();
      uint16_t setBusUnderVoltage(float busUnderVoltage);
      uint16_t setBusOverVoltage(float busOverVoltage);
  
      uint8_t  setTemperatureLimit();
      int16_t  setTemperatureLimit(float temperatureLimit);
  
      uint16_t setPowerLimit();
      uint16_t setPowerLimit(float power);
  
      int16_t  getShuntUnderVoltage();
      int16_t  getShuntOverVoltage();
  
      uint16_t getBusUnderVoltage();
      uint16_t getBusOverVoltage();
  
      int16_t  getTemperatureLimit(); 

      uint8_t  writeRegister(uint8_t registerAddress, int16_t value);
      int32_t  readRegister(uint8_t registerAddress);

      float    getShuntVoltage();
      float    getBusVoltage();
      float    getCurrent();  
      float    getPower();
      float    getTemperature();
  
    private:
      uint8_t           _deviceAddress;
      float             _rShunt;
      float             _maximumCurrent;
      float             _currentLsbMin;
      uint8_t           _rangeAdc;
      float             _shuntAdcResolution;
      TwoWire *         _wire; //The generic connection to user's chosen I2C hardware
  };
  
  #endif
