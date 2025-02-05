/**
    *******************************************************************************************
  * @file           : MAX31875.h
  * @brief          : MAX31875Rx Library (Where x = 0-7)
    *******************************************************************************************

  * The MAX31875 is a ±1°C-accurate local temperature sensor with I2C/SMBus interface. The 
  * combination of tiny package and excellent temperature measurement accuracy makes this 
  * product ideal for a variety of equipment
  * 
  * The I2C/SMBus-compatible serial interface accepts standard write byte, read byte, send
  * byte and receive byte commands to read the temperature data and configure the behavior
  * of the sensor. Bus timeout resets the interface if the clock is low for more than 30ms
  * (nominal). PEC helps to avoid communication errors when used with a master that supports
  * this feature
  * The MAX31875 is available in a 4-bump, wafer-level package (WLP) and operates over -50°C
  * to +150°C temperature range.
  *   
  * @details
  * Tiny 0.84mm x 0.84mm x 0.35mm WPL
  * Excellent temperature accuracy
  * - ±1.75°C from -40°C to +145°C
  * - ±1°C from -0°C to +70°C
  * <10µA Average Power Supply Current
  * Selectable timeout Prevents Bus Lockup
  * I2C and SMBus support
  * Selectable PEC for Reliable Communications
  * +1.6V to +3.6V Power Supply Voltage
  *******************************************************************************************
  */
#ifndef INC_MAX31875_H_
#define INC_MAX31875_H_

#define MAX31875_ADDRESS                         0x48 //For MAX31875R0
#define MAX31875_TRIALS                          5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// REGISTERS
#define MAX31875_TEMPERATURE_REGISTER            0x00 // Reset state = 0000h - 0°C 
#define MAX31875_CONFIGURATION_REGISTER          0x01 // Reset state = 0040h - n/a
#define MAX31875_TEMPERATURE_HYST_REGISTER       0x02 // Reset state = 4B00h - 75°C
#define MAX31875_TEMPERATURE_OS_REGISTER         0x03 // Reset state = 5000h - 80°C

#define MAX31875_8_BITS_RES                      1.0
#define MAX31875_9_BITS_RES                      0.5
#define MAX31875_10_BITS_RES                     0.25
#define MAX31875_12_BITS_RES                      0.0625

// Helps to reduce average supply current when continuous
// conversions are not necessary
typedef enum
{
  MAX31875_ONESHOT_ENABLE                        = 0x0000,
  MAX31875_ONESHOT_DISABLED                      = 0x0001,
}MAX31875OneShot_t;

typedef enum
{
  MAX31875_0_DOT_25_SAMPLES_PER_SECOND           = 0x0000, // A conversion is started every 4 seconds - Default
  MAX31875_1_SAMPLE_PER_SECOND                   = 0x0002, // A conversion is started every 1 second
  MAX31875_4_SAMPLES_PER_SECOND                  = 0x0004, // A conversion is started every 0.25 seconds
  MAX31875_8_SAMPLES_PER_SECOND                  = 0x0006, // A conversion is started every 0.125 seconds
}MAX31875ConversionRate_t;

// Packet Error Checking. When enabled a PEC bytes is appended
// to the end of each message transfer.
// - The master transmits the PEC byte after a Write transaction
// - the MAX31875 transmits the PEC byte after a Read transaction.
typedef enum
{
  MAX31875_PEC_ENABLED                           = 0x0008,
  MAX31875_PEC_DISABLED                          = 0x0000,
}MAX31875Pec_t;

// Bus timeout resets the I2C interface when SCL is low for
// more than 30ms (nominal)
typedef enum
{
  MAX31875_TIMEOUT_ENABLED                       = 0x0000,
  MAX31875_TIMEOUT_DISABLED                      = 0x0010,
}MAX31875Timeout_t;

typedef enum
{
  MAX31875_8_BITS_RESOLUTION                    = 0x0000,
  MAX31875_9_BITS_RESOLUTION                    = 0x0020,
  MAX31875_10_BITS_RESOLUTION                   = 0x0040, // Default
  MAX31875_12_BITS_RESOLUTION                   = 0x0060,

}MAX31875Resolution_t;

// When D7 is 0 (normal format), the data format is two's complement with a
// range of -128°C to (128°C - 1LSB)
// When D7 is 1 (extended temperature format) the device allows temperatures
// as high as 150°C
typedef enum
{
  MAX31875_NORMAL_TEMPERATURE_FORMAT            = 0x0000,
  MAX31875_EXTENDED_TEMPERATURE_FORMAT          = 0x0080,
}MAX31875DataFormat_t;

// When enabled, supply current is reduced to 1µA or less
// When disabled, continuous conversion mode is enabled at the rate defined by
// MAX31875ConversionRate_t
typedef enum
{
  MAX31875_SHUNTDOWN_ENABLED                   = 0x0100,
  MAX31875_CONTINUOUS_CONVERSION_ENABLED       = 0x0000,
}MAX31875ShuntDown_t;

typedef enum
{
  MAX31875_COMPARATOR_MODE                     = 0x0000,
  MAX31875_INTERRUPT_MODE                      = 0x0200,
}MAX31875CompInit_t;

// Select how many consecutive overtemperature faults must occur
// before an overtemperature fault is inidcated in the OT Status bit
typedef enum
{
  MAX31875_1_FAULT                             = 0x0000, // Default
  MAX31875_2_FAULTS                            = 0x0800,  
  MAX31875_4_FAULTS                            = 0x1000,
  MAX31875_6_FAULTS                            = 0x1800,
}MAX31875FaultQueue_t;

class MAX31875{
  public:
    MAX31875(); // Constructor. It must be defined with the same name as the class
    uint8_t  init(uint8_t devAddress, TwoWire *wire = &Wire);
    uint8_t  customInit(uint8_t devAddress, MAX31875OneShot_t oneShot, MAX31875ConversionRate_t convRate, MAX31875Pec_t pec, MAX31875Timeout_t timeout, MAX31875Resolution_t resolution, MAX31875DataFormat_t dataFormat, MAX31875ShuntDown_t shutdownMode, MAX31875CompInit_t compInit, MAX31875FaultQueue_t faults, TwoWire *wire = &Wire);
    uint8_t  writeRegister(uint8_t registerAddress, uint16_t value);
    int16_t  readRegister(uint8_t registerAddress);
    uint16_t getConfig();
    void     setLowLimit(int16_t lowLimit);
    void     setHighLimit(int16_t highLimit);
    float    getTemperature();
    bool     getOverTemperatureBit();
  private:
    uint8_t _deviceAddress = MAX31875_ADDRESS;
    int16_t  setTemperatureLimit(int16_t tempLimit);
    MAX31875Resolution_t  _resolution;
    float checkTemp(int16_t value);
    TwoWire * _wire; //The generic connection to user's chosen I2C hardware
};

#endif